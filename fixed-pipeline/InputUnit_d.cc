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
#include "mem/ruby/network/garnet/fixed-pipeline/InputUnit_d.hh"
#include "mem/ruby/network/garnet/fixed-pipeline/Router_d.hh"
#include "mem/ruby/network/garnet/fixed-pipeline/SMART_Coordinate.hh"


using namespace std;
using m5::stl_helpers::deletePointers;

void
InputUnit_d::increment_credit(int in_vc, bool free_signal, Cycles curTime)
{
    flit_d *t_flit = new flit_d(in_vc, free_signal, curTime);
    creditQueue->insert(t_flit);
    m_credit_link->scheduleEventAbsolute(m_router->clockEdge(Cycles(1)));
}

InputUnit_d::InputUnit_d(int id, Router_d *router) : Consumer(router)
{
    m_id = id;
    m_router = router;
    m_num_vcs = m_router->get_num_vcs();
    m_vc_per_vnet = m_router->get_vc_per_vnet();

    m_num_buffer_reads.resize(m_num_vcs/m_vc_per_vnet);
    m_num_buffer_writes.resize(m_num_vcs/m_vc_per_vnet);
    for (int i = 0; i < m_num_buffer_reads.size(); i++) {
        m_num_buffer_reads[i] = 0;
        m_num_buffer_writes[i] = 0;
    }

    creditQueue = new flitBuffer_d();
    // Instantiating the virtual channels
    m_vcs.resize(m_num_vcs);
    for (int i=0; i < m_num_vcs; i++) {
        m_vcs[i] = new VirtualChannel_d(i);
    }
}

InputUnit_d::~InputUnit_d()
{
    delete creditQueue;
    deletePointers(m_vcs);
}

void 
InputUnit_d::wakeup()
{
    flit_d *t_flit;
    if (m_in_link->isReady(m_router->curCycle())) {

        t_flit = m_in_link->consumeLink();

		if( t_flit->get_stage().first == SMART_LT_ )
			fatal("Normal Input unit connected to smart link !");

		
        int vc = t_flit->get_vc();

		if( m_router->GetSmartCoordinate() && m_router->GetSmartCoordinate()->isSmartFlit(t_flit,m_router->getID(),m_id,vc) )
		{
			inform("router %d, inputunit:%d, invc:%d, flit_id: %d, is_head:%d is inserted into smart_shadow_buffer",
					m_router->getID(), m_id, vc, t_flit->get_id(), 
					(t_flit->get_type()==HEAD_) || ( t_flit->get_type() == HEAD_TAIL_ )
					);
					
			int inport_vc_index = m_id * m_num_vcs + vc;
			if( ( t_flit->get_type() == HEAD_ ) || ( t_flit->get_type() == HEAD_TAIL_ ) )
			{
				assert(m_router->m_smart_state[inport_vc_index] == smart_vc_idle);
				
				m_router->m_smart_dest_router_index[inport_vc_index] = m_router->GetSmartCoordinate()->getFlitSmartDestRouterIndex(t_flit);
				m_router->m_smart_state[inport_vc_index] = smart_vc_wait_for_vc_grant;
			}
			if( m_router->getID() == 57 && m_id == 5 && vc==9 && t_flit->get_id() == 1)
			{
				inform("for checking mem error!");
				cout<<"pnt:"<<(m_router->m_smart_in_buffer[inport_vc_index])<<endl;
			}
			
			m_router->smartLinkActivate();
			if(inport_vc_index >= m_router->m_smart_in_buffer.size() )
				fatal("indexing to inport_VC:%d >= max:%d",inport_vc_index,m_router->m_smart_in_buffer.size());
			
			m_router->m_smart_in_buffer[inport_vc_index]->insert(t_flit);
		}
		else
		{
		    if ((t_flit->get_type() == HEAD_) ||
		       (t_flit->get_type() == HEAD_TAIL_)) {

		        assert(m_vcs[vc]->get_state() == IDLE_);
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

uint32_t
InputUnit_d::functionalWrite(Packet *pkt)
{
    uint32_t num_functional_writes = 0;
    for (int i=0; i < m_num_vcs; i++) {
        num_functional_writes += m_vcs[i]->functionalWrite(pkt);
    }

    return num_functional_writes;
}

void
InputUnit_d::resetStats()
{
    for (int j = 0; j < m_num_buffer_reads.size(); j++) {
        m_num_buffer_reads[j] = 0;
        m_num_buffer_writes[j] = 0;
    }
}
