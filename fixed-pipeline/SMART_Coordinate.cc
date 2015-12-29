/*
 * Copyright (c) 2015 Princeton University
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are
 * met: redistributions of source code must retain the above copyright
 * notice, this list of conditions and the following disclaimer;
 * redistributions in binary form must reproduce the above copyright
 * notice, this list of conditions and the following disclaimer in the
 * documentation and/or other materials provided with the distribution;
 * neither the name of the copyright holders nor the names of its
 * contributors may be used to endorse or promote products derived from
 * this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 * A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 * OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 * DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 * THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 * Authors: Hongce Zhang
 */
 
#include "base/stl_helpers.hh"
#include "base/cast.hh"
#include "mem/ruby/network/garnet/fixed-pipeline/InputUnit_d.hh"
#include "mem/ruby/network/garnet/fixed-pipeline/Router_d.hh"
#include "mem/ruby/slicc_interface/Message.hh"
#include "mem/ruby/slicc_interface/AbstractController.hh"
#include "mem/ruby/network/garnet/fixed-pipeline/SmartInputUnit_d.hh"
#include "mem/ruby/network/garnet/fixed-pipeline/SMART_Coordinate.hh"

using namespace std;
using m5::stl_helpers::deletePointers;

SMART_Coordinate:: SMART_Coordinate(const Params *p)
    : ClockedObject(p),Consumer(this)
{
	//number of routers in group
	// router's input buffer
	
	for (vector<Router_d *>::const_iterator i = p->rlist.begin();
         i != p->rlist.end(); ++i) {
        Router_d* router = safe_cast<Router_d *>(*i);
        m_router_list.push_back(router);
		router_seq.push_back(router->getID() );

		num_vcs = router->get_num_vcs();
		num_vcs_per_vnet = router->get_vc_per_vnet();
		assert(num_vcs != 0);
		assert(num_vcs_per_vnet != 0);
    }
	for (vector<SmartConfig *>::const_iterator i = p->layerlist.begin();
		 i != p->layerlist.end(); ++i) {
			 SmartConfig * sg = safe_cast<SmartConfig *>(*i);
			 route_compute_list.push_back(sg);
		 }
	if( m_router_list.size() != route_compute_list.size() )
		fatal("Size mismatch of SMART router and route info");
	
	int router_num = m_router_list.size();
/*
	inform("router_num:%d",router_num);
	for(int i=0;i<router_num;i++)
		inform("%d ",m_router_list[i]->getID());
		*/
	
	m_smart_router_in_buffers.resize(router_num);
	m_output_router_input_unit.resize(router_num);
	ovc_free.resize(router_num);
	vc_credits_for_smart_in_unit.resize(router_num);
	
	VCA_last_router_round = 0;
	VCA_last_inport_per_router.resize(router_num);
	VCA_last_vc_round_per_router.resize(router_num);
	
	LA_last_router_round = 0;
	LA_last_vc_round_per_router.resize(router_num);
	LA_last_inport_per_router.resize(router_num);
	
	linkIdleCycle = 0;
	linkWaitforVACycle = 0;
}


int SMART_Coordinate::getSmartConfigIndexOfDestRoute(int rID, int &iter_post)
{
	for(int layer_iter=0;layer_iter<route_compute_list.size(); ++layer_iter)
	{
		for(int r_iter=0;r_iter < route_compute_list[layer_iter]->m_smart_router_list.size(); ++r_iter)
			if( route_compute_list[layer_iter]->m_smart_router_list[r_iter]->getID()  == rID)
			{
				iter_post = r_iter;
				return layer_iter;
			}
	}
	fatal("SMART_Coordinate did not found rID:%d",rID);
	return -1;
}

bool SMART_Coordinate::isSmartFlit(flit_d *in_flit,uint32_t rID,int Inport_id,int vc)
{
	int srcRouterIndex = router_id_to_index(rID);
	Router_d * CurRouter = m_router_list[srcRouterIndex];
	int Inport_VC_index = Inport_id*num_vcs + vc;
	if( CurRouter->m_smart_state[Inport_VC_index] != smart_vc_idle )
	{
		if( in_flit->get_type() == HEAD_ || in_flit->get_type() == HEAD_TAIL_ )
			fatal("Smart coordinate: new head arrived on a busy input vc");
		return true;
	}
	
	
	
	MsgPtr msg_ptr = in_flit->get_msg_ptr();
    Message *net_msg_ptr = msg_ptr.get();
    NetDest msg_destination = net_msg_ptr->getDestination();
	
	vector<unsigned int> dest_list = msg_destination.getAllDest();
	if( dest_list.size() != 1  )
		fatal("Smart unable to process destination more than 1");
	
	int dest_router_id = dest_list[0] - MachineType_base_number((MachineType)1);
	
	int origin_post;
	int dest_post;
	
	int input_layer = getSmartConfigIndexOfDestRoute(rID,origin_post);
	int output_layer = getSmartConfigIndexOfDestRoute(dest_router_id,dest_post);
	if( input_layer == output_layer )
		return false;
	if( origin_post != dest_post)
		return false;
	return true;
	
}

int SMART_Coordinate::getFlitSmartDestRouterIndex(flit_d *in_flit)
{
	MsgPtr msg_ptr = in_flit->get_msg_ptr();
    Message *net_msg_ptr = msg_ptr.get();
    NetDest msg_destination = net_msg_ptr->getDestination();
	int tmp_post;
	
	vector<unsigned int> dest_list = msg_destination.getAllDest();
	if( dest_list.size() != 1  )
		fatal("Smart unable to process destination more than 1");
	int dest_router_id = dest_list[0] - MachineType_base_number((MachineType)1);
	int output_layer = getSmartConfigIndexOfDestRoute(dest_router_id,tmp_post);
	return output_layer;
}

int SMART_Coordinate::router_id_to_index(uint32_t rID)
{
	vector<uint32_t>::const_iterator iter= std::find(router_seq.begin(),router_seq.end(),rID);
	if( iter ==  router_seq.end() )
		fatal("rID : %d 's index not found!",rID);
	
	return iter - router_seq.begin();
}
uint32_t SMART_Coordinate::router_index_to_id(int index)
{
	if( index >= router_seq.size() )
		fatal("Index to [%d] router in smart coor!",index);
	return router_seq[index];
}

void SMART_Coordinate::init()
{
	// this is self init
	// nothing need to do, except initalizin stats
	linkIdleCycle = 0;
	linkWaitforVACycle = 0;
	
	
}
// this depends on the number of input units
void SMART_Coordinate::init(uint32_t routerID, GarnetNetwork_d *Network_ptr) // called by each router to allocate its arb resources
{
	
	int routerIndex = router_id_to_index(routerID);
	//Router_d * CurRouter = m_router_list[ routerIndex ];
	//m_smart_router_in_buffers[ routerIndex ] already initialized in global_make_link
	
	//inform("SMART_Coor: router %d init with index %d",routerID,routerIndex);

	ovc_free[routerIndex].resize(num_vcs);
	vc_credits_for_smart_in_unit[routerIndex].resize(num_vcs);
	for(int vc_iter=0;vc_iter<num_vcs;vc_iter++)
	{
		int initialCredit = (Network_ptr->get_vnet_type(vc_iter) == DATA_VNET_) ? 
							(Network_ptr->getBuffersPerDataVC() ) :
							(Network_ptr->getBuffersPerCtrlVC() );
		vc_credits_for_smart_in_unit[routerIndex][vc_iter] = initialCredit;
		ovc_free[routerIndex][vc_iter] = true;
		//inform("vc %d initial credit:%d", vc_iter, initialCredit);
	}
	VCA_last_inport_per_router[routerIndex] = 0;
	VCA_last_vc_round_per_router[routerIndex] = 0;
	
	LA_last_vc_round_per_router[routerIndex] = 0;
	LA_last_inport_per_router[routerIndex] = 0;
	//FIXME:
}

void SMART_Coordinate::VCAllocate()
{
	int n_router = m_router_list.size();
	
	int router_id = VCA_last_router_round;
	for(int router_iter=0;router_iter<n_router;router_iter++)
	{
		Router_d * Cur_Router = m_router_list[router_id];
		
		
		int vc_id = VCA_last_vc_round_per_router[router_id];
		int inport_id = VCA_last_inport_per_router[router_id];
		int total_fb_size = Cur_Router->m_smart_in_buffer.size();
		
		for(int vc_iport_iter = 0;vc_iport_iter < total_fb_size;vc_iport_iter ++)
		{
			
			int smart_buffer_index = inport_id * num_vcs + vc_id;
			bool out_vc_found = false;
			// choose a grant need
			if(Cur_Router->m_smart_state[smart_buffer_index] == smart_vc_wait_for_vc_grant)
			{
				// choose an out vc
				int destRIndex = Cur_Router->m_smart_dest_router_index[smart_buffer_index];
				int o_vnet = get_vnet(vc_id);
				for(int ovc_iter = o_vnet*num_vcs_per_vnet;ovc_iter <o_vnet*num_vcs_per_vnet + num_vcs_per_vnet;ovc_iter++)
				{
					if(ovc_free[destRIndex][ovc_iter])
					{
						// then we allocate [destRIndex].[ovc_iter] to Cur_Router's
						ovc_free[destRIndex][ovc_iter] = false;
						Cur_Router->m_smart_state[smart_buffer_index] = smart_vc_granted;
						Cur_Router->m_smart_out_vc[smart_buffer_index] = ovc_iter;
						Cur_Router->m_smart_credit[smart_buffer_index] = vc_credits_for_smart_in_unit[destRIndex][ovc_iter];
						
						// debug inform here
						/* inform("grant ovc:%d at DestId:%d to input:%d,vc:%d @srcID:%d", ovc_iter, 
							router_index_to_id(destRIndex), smart_buffer_index/num_vcs,
							smart_buffer_index%num_vcs, router_index_to_id(router_id)
							);*/
						
						out_vc_found = true;
						break;
					}
				}
			}
			
			if( out_vc_found )
				break;
			
			vc_id ++;
			if(vc_id >= num_vcs) 
			{
				vc_id = 0;
				inport_id ++;
				if(inport_id >= Cur_Router->get_num_inports() )
					inport_id = 0;
			}
		}
		VCA_last_vc_round_per_router[router_id] ++; // = vc_id
		VCA_last_inport_per_router[router_id] ++; // = inport_id
		if(VCA_last_vc_round_per_router[router_id] >= num_vcs )
			{
				VCA_last_vc_round_per_router[router_id] = 0;
			}
		if(VCA_last_inport_per_router[router_id] >= Cur_Router->get_num_inports() )
			VCA_last_inport_per_router[router_id] = 0;
		
		
		router_id ++;
		if(router_id>=n_router)
			router_id = 0;
	}
	VCA_last_router_round ++; // next time start from the next router
	if(VCA_last_router_round >= m_router_list.size() )
		VCA_last_router_round = 0;
}

bool SMART_Coordinate::waiting_for_VA_grant()
{
	for(int router_iter = 0; router_iter< m_router_list.size(); ++router_iter)
	{
		Router_d* CurRouter = m_router_list[router_iter];
		for(int inport_vc_iter = 0; inport_vc_iter< CurRouter->m_smart_state.size(); ++inport_vc_iter)
		{
			if( CurRouter->m_smart_state[inport_vc_iter] == smart_vc_wait_for_vc_grant )
				return true;
		}
	}
	return false;
}

void inline SMART_Coordinate::sendAFlit(Router_d * Cur_Router, int inport_id, int vc_id, int smart_buffer_index) // only source specified, destination could be determined
{
	int destRIndex = Cur_Router->m_smart_dest_router_index[smart_buffer_index];
	int output_vc = Cur_Router->m_smart_out_vc[smart_buffer_index];
	
	flit_d * t_flit = Cur_Router->m_smart_in_buffer[smart_buffer_index]->getTopFlit();
	t_flit->set_vc(output_vc);
	t_flit->advance_stage(SMART_LT_, curCycle()+Cycles(1) ); // FIXME: time incorrect!!!
	t_flit->set_time( curCycle()+Cycles(1) );
	m_smart_router_in_buffers[destRIndex]->insert(t_flit);
	m_output_router_input_unit[destRIndex]->scheduleEventAbsolute( curCycle()+Cycles(1) );
	
	vc_credits_for_smart_in_unit[destRIndex][output_vc] --;		
	Cur_Router->m_smart_credit[smart_buffer_index] = vc_credits_for_smart_in_unit[destRIndex][output_vc];
	
	// update backward credit
	InputUnit_d * iunit = Cur_Router->get_inputUnit_ref()[inport_id];
	assert(iunit != safe_cast<InputUnit_d *>(Cur_Router->getSmartInputUnit()) );
	
	if(t_flit->get_type() == TAIL_ ||  t_flit->get_type() == HEAD_TAIL_ )
	{
		iunit->increment_credit(vc_id, true,curCycle() );
		Cur_Router->m_smart_state[smart_buffer_index] = smart_vc_idle;
	}
	else
	{
		iunit->increment_credit(vc_id, false,curCycle() );
	}
	
	/*
	inform("Link Occupied to ovc:%d at DestId:%d by input:%d,vc:%d @srcID:%d", 
			output_vc, router_index_to_id(destRIndex), 
			smart_buffer_index/num_vcs,
			smart_buffer_index%num_vcs, Cur_Router->getID()
			);
			
	inform("flit_id: %d, is_head:%d, is_tail:%d is tranversing smart link",
	 t_flit->get_id(), (t_flit->get_type()==HEAD_) || ( t_flit->get_type() == HEAD_TAIL_ ),
	 (t_flit->get_type()==TAIL_) || ( t_flit->get_type() == HEAD_TAIL_ )	); */
}


void SMART_Coordinate::linkAllocte()
{
	
	int n_router = m_router_list.size();
	
	int router_id = LA_last_router_round;
	int flit_sent = 0;
	
	vector<bool> linkOccupied(n_router,false);
	
	for(int router_iter=0; router_iter<n_router; router_iter++)
	{
		Router_d * Cur_Router = m_router_list[router_id];
		Cycles self_cycle = curCycle();
		Cycles router_cycle = Cur_Router->curCycle();
		if( self_cycle != router_cycle )
			fatal("Smart Coordinate cycle mismatch between smart and router!");
		
		
		int vc_id = LA_last_vc_round_per_router[router_id];
		int inport_id = LA_last_inport_per_router[router_id];
		int total_fb_num = Cur_Router->m_smart_in_buffer.size(); // num_vcs * router's inport # // we need to assert this somewhere
		
		for(int vc_iport_iter = 0;vc_iport_iter < total_fb_num;vc_iport_iter ++)
		{
			
			int smart_buffer_index = inport_id * num_vcs + vc_id;
			// choose a grant need
			if(Cur_Router->m_smart_state[smart_buffer_index] == smart_vc_granted && ! Cur_Router->m_smart_in_buffer[smart_buffer_index]->isEmpty())
			{
				// update smart_In_unit_credit
				int output_vc = Cur_Router->m_smart_out_vc[smart_buffer_index];
				int destRIndex = Cur_Router->m_smart_dest_router_index[smart_buffer_index];
				
				Cur_Router->m_smart_credit[smart_buffer_index] = vc_credits_for_smart_in_unit[destRIndex][output_vc];
				
				if( Cur_Router->m_smart_credit[smart_buffer_index] > 0 && !intersect(linkOccupied,router_id,destRIndex) )
				{
					// ok to send
					setOccupied(linkOccupied,router_id,destRIndex);
					
					sendAFlit(Cur_Router, inport_id,vc_id, smart_buffer_index );
					
					flit_sent ++;
					
					break;
				}
			}	
			
			vc_id ++;
			if(vc_id >= num_vcs) 
			{
				vc_id = 0;
				inport_id ++;
				if(inport_id >= Cur_Router->get_num_inports() )
					inport_id = 0;
			}
		}
	#ifdef	THE_OTHER_VA_METHOD
		LA_last_vc_round_per_router[router_id] ++; // = vc_id + 1
		LA_last_inport_per_router[router_id] ++; // = inport_id
		if(LA_last_vc_round_per_router[router_id] >= num_vcs )
			{
				LA_last_vc_round_per_router[router_id] = 0;
			}
		if(LA_last_inport_per_router[router_id] >= Cur_Router->get_num_inports() )
			LA_last_inport_per_router[router_id] = 0;
	#else
		/*
		warn("changing LA_last_vc_round_per_router[%d] %d -> %d",router_id, LA_last_vc_round_per_router[router_id], vc_id + 2);
		warn("changing LA_last_inport_per_router  [%d] %d -> %d",router_id, LA_last_inport_per_router  [router_id], inport_id);
		*/
		LA_last_vc_round_per_router[router_id] = vc_id + 2;
		if(LA_last_vc_round_per_router[router_id] >= num_vcs)
			LA_last_vc_round_per_router[router_id] = 0;
		LA_last_inport_per_router[router_id] = inport_id;
	#endif		
		
		router_id ++;
		if(router_id>=n_router)
			router_id = 0;
	}
	if(flit_sent == 0)
	{
		// stats: +1
		linkIdleCycle ++;
		if( waiting_for_VA_grant() )
		{
			// stats: va limit: +1
			linkWaitforVACycle ++;
		}
	}
	assert(flit_sent<n_router*2);
	
	FlitPerCycle[flit_sent] ++;
	
	LA_last_router_round ++; // next time start from the next router // = router_id
	if(LA_last_router_round >= m_router_list.size() )
		LA_last_router_round = 0;

}
void SMART_Coordinate::wakeup()
{
	//inform("smart coor wakeup");
	// vc arb
	VCAllocate();
	//link arb & tranverse
	linkAllocte();	
	
	check_for_wakeup();
}

void SMART_Coordinate::check_for_wakeup()
{
	for(int r_iter = 0;r_iter < m_router_list.size(); r_iter++)
	{
		Cycles nextCycle = m_router_list[r_iter]->curCycle() + Cycles(1);
		for(int vc_inport_iter=0; vc_inport_iter < m_router_list[r_iter]->get_num_inports() * num_vcs; vc_inport_iter++ )
			if( m_router_list[r_iter]->m_smart_in_buffer[vc_inport_iter]->isReady(nextCycle) )
			{	
				m_router_list[r_iter]->smartLinkActivate();
				return;
			}
	}
}


void SMART_Coordinate::global_make_link()
{
	for(int i=0;i< m_router_list.size(); ++i)
	{
		flitBuffer_d *fb_in = new flitBuffer_d();
		m_router_list[i]->SetSmartCoordinate(this); // this must be the first
		
		m_router_list[i]->AddSmartInputUnit( fb_in ); // because this depends on smartcoor
		m_smart_router_in_buffers[i] = fb_in;
		m_output_router_input_unit[i] = m_router_list[i]->getSmartInputUnit();
	}
}

void SMART_Coordinate::UpdateVC(uint32_t rID,int vc,bool isfree)
{
	int routerIndex = router_id_to_index(rID);
	assert(vc<num_vcs);
	
	if( ovc_free[routerIndex][vc] != false)
		fatal("freeing an unallocated ovc in rID:%d,rIndex:%d,vc:%d",rID,routerIndex,vc);
	
	vc_credits_for_smart_in_unit[routerIndex][vc] ++;
	if(isfree)
		ovc_free[routerIndex][vc] = true;
}

void SMART_Coordinate::regStats()
{
	linkIdleCycle.name(name()+".linkIdleCycle").flags(Stats::nonan);
	linkWaitforVACycle.name(name()+".linkWaitforVACycle").flags(Stats::nonan);	
	FlitPerCycle.init( m_router_list.size()*2 ).name(name() + ".cycles_with_diff_flits").flags(Stats::pdf | Stats::total | Stats::oneline);
}

SMART_Coordinate::~SMART_Coordinate()
{
	deletePointers(m_smart_router_in_buffers);
	//cout<<"smart stats linkIdleCycle: "<<linkIdleCycle<<"\tlinkWaitForVA: "<<linkWaitforVACycle<<endl;
}


bool SMART_Coordinate::intersect(vector<bool> linkOccupied, int srcIndex, int destIndex)
{
	assert(srcIndex  >=0 && srcIndex  < linkOccupied.size() );
	assert(destIndex >=0 && destIndex < linkOccupied.size() );	
	for (int iter = min(srcIndex,destIndex) ; iter <= max(srcIndex,destIndex); ++iter)
	{
		if( linkOccupied[iter] )
			return true;
	}
	return false;
}

void SMART_Coordinate::setOccupied( vector<bool> linkOccupied, int srcIndex, int destIndex )
{
	assert(srcIndex  >=0 && srcIndex  < linkOccupied.size() );
	assert(destIndex >=0 && destIndex < linkOccupied.size() );	
	for (int iter = min(srcIndex,destIndex) ; iter <= max(srcIndex,destIndex); ++iter)
	{
		assert( !linkOccupied[iter] );
		linkOccupied[iter] = true;
	}
}


SMART_Coordinate* 
SMART_CoordinateParams::create()
{
    return new SMART_Coordinate(this);
}