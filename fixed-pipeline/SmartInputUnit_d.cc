/*
 * Copyright (c) 2008 Princeton University
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
 * Authors: Niket Agarwal
 */

#include "base/stl_helpers.hh"
#include "mem/ruby/network/garnet/fixed-pipeline/SmartInputUnit_d.hh"
#include "mem/ruby/network/garnet/fixed-pipeline/SMART_Coordinate.hh"
#include "mem/ruby/network/garnet/fixed-pipeline/Router_d.hh"

// smart input unit does not consume from links but directly from FlitBuffers

using namespace std;

void  
SmartInputUnit_d::increment_credit(int in_vc, bool free_signal, Cycles curTime)
{
	smart_coor_pnt->UpdateVC(m_router->getID(),in_vc,free_signal);
} // only need tell the output, when freeing

void 
SmartInputUnit_d::wakeup()
{
    flit_d *t_flit; // here we get it directly from buffer not links
    while (m_in_flit_buffer->isReady(m_router->curCycle())) {
        t_flit = m_in_flit_buffer->getTopFlit();

/*
		t_flit->router_id_passed.push_back(m_router->getID());
				t_flit->router_cycle_passed.push_back(m_router->curCycle());
				if(m_router->curCycle()-t_flit->router_cycle_passed[0]>25) {
				cout<<"FLIT route:";
				for(int i=0;i<t_flit->router_id_passed.size();i++)
					cout<<"("<<t_flit->router_id_passed[i]<<" : "<<t_flit->router_cycle_passed[i]<<")\t";
				cout<<endl;
					}


 		if(m_in_flit_buffer->isReady(m_router->curCycle())) {
			//warn("multiple filt arrive at the same SmartInput-port! ");
			//cout<<(*t_flit)<<endl;
			//cout<<(*(m_in_flit_buffer->peekTopFlit()))<<endl;
 			}*/
	
		if( t_flit->get_stage().first != SMART_LT_ )
			fatal("Smart Input unit connected to normal link !");

		// move back to normal 
		t_flit->advance_stage(LT_,m_router->curCycle() );
		
        int vc = t_flit->get_vc();

		if( t_flit->get_time() < m_router->curCycle() )
			{
			warn("SMART UNIT:time discrepency: flit:%d, curTime:%d", t_flit->get_time(),m_router->curCycle());
			if( m_router->curCycle() - t_flit->get_time() > 3)
				fatal("SMART UNIT:Strong discrepency!");
			}
/*		
		inform("router %d, smart_inputunit:%d, invc:%d, flit_id: %d, is_head:%d is received",
					m_router->getID(), m_id, vc, t_flit->get_id(), 
					(t_flit->get_type()==HEAD_) || ( t_flit->get_type() == HEAD_TAIL_ )  )
					;*/
					
		if( smart_coor_pnt->isSmartFlit(t_flit,m_router->getID() ,m_id,vc) )
		{
			warn("sending smart flit to smart link again!");
			int inport_vc_index = m_id * m_num_vcs + vc;
			if( ( t_flit->get_type() == HEAD_ ) || ( t_flit->get_type() == HEAD_TAIL_ ) )
			{
				m_router->m_smart_dest_router_index[inport_vc_index] = smart_coor_pnt->getFlitSmartDestRouterIndex(t_flit);
				m_router->m_smart_state[inport_vc_index] = smart_vc_wait_for_vc_grant;
			}
			
			m_router->smartLinkActivate();
			m_router->m_smart_in_buffer[inport_vc_index]->insert(t_flit);
		}
		else
		{
				
			if ((t_flit->get_type() == HEAD_) ||  (t_flit->get_type() == HEAD_TAIL_)) 
			{

				/*
				if( m_vcs[vc]->get_state() != IDLE_ )
					{
						warn("smart inputunit:%d router:%d, vc:%d is not released", m_id, m_router->getID(), vc);
					}
				*/
				
				assert(m_vcs[vc]->get_state() == IDLE_);
				//inform("First occupy inputunit:%d router:%d, vc:%d ", m_id, m_router->getID(), vc);
					
				// Do the route computation for this vc, and this will schedule for VCalloc
				// No need to do it twice 
				m_router->route_req(t_flit, this, vc);

				m_vcs[vc]->set_enqueue_time(m_router->curCycle());
			} else {
				t_flit->advance_stage(SA_, m_router->curCycle());
				// Changing router latency to 2 cycles. Input Unit takes 1 cycle for wakeup.
				// VCalloc, SWalloc, Sw-Xfer and output scheduling takes 1 cycle. The original
				// design schedules VCallocator for head flit, and Swalloc for non-head flit.
				// VCalloc now calls SWalloc directly instead of scheduling it for the next cycle,
				// hence we should not allocate SWalloc, otherwise it might get called twice, once
				// by the scheduler and once by VCalloc.
				m_router->vcarb_req();
			}
			// write flit into input buffer
			m_vcs[vc]->insertFlit(t_flit);

			int vnet = vc/m_vc_per_vnet;
			// number of writes same as reads
			// any flit that is written will be read only once
			m_num_buffer_writes[vnet]++;
			m_num_buffer_reads[vnet]++;
		}
		

    }
}
