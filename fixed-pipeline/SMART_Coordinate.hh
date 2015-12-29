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

#ifndef __MEM_RUBY_NETWORK_GARNET_FIXED_PIPELINE_SMART_COORDINATE_HH__
#define __MEM_RUBY_NETWORK_GARNET_FIXED_PIPELINE_SMART_COORDINATE_HH__

#include <iostream>
#include <vector>

#include "mem/ruby/common/Consumer.hh"
#include "mem/ruby/network/garnet/fixed-pipeline/flit_d.hh"
#include "mem/ruby/network/garnet/fixed-pipeline/flitBuffer_d.hh"
#include "mem/ruby/network/garnet/NetworkHeader.hh"
#include "mem/ruby/network/garnet/fixed-pipeline/NetworkLink_d.hh"
#include "mem/ruby/network/garnet/fixed-pipeline/SmartConfig.hh"
#include "params/SMART_Coordinate.hh"

// This must be visible from python side

class Router_d;
class OutputUnit_d;
class SmartInputUnit_d;
class InputUnit_d;

class SMART_Coordinate : public ClockedObject, public Consumer
{
  public:
	typedef SMART_CoordinateParams Params;
    SMART_Coordinate(const Params *p);
    ~SMART_Coordinate();
    
    void wakeup();
	void virtual init();
    void init(uint32_t routerID, GarnetNetwork_d *Network_ptr);
    void check_for_wakeup();
    
    void print(std::ostream& out) const {};

	void global_make_link();
	
	// void LA_global_request(uint32_t routerId, Tick time, uint32_t destRouterId);
	
	// bool isGrant(int routerId, Tick time);
	// bool needStall(int routerId, Tick time); 
	
	//void SendFlit(flit_d *in_flit,int smartDest);
	
	bool isSmartFlit(flit_d *in_flit,uint32_t rID,int Inport_id,int vc); // once smartFlitBuffer for [inport][vc] is allocte and not free 
															 // it will always be // incase the destination is not correct
															 // better check it 
	int getFlitSmartDestRouterIndex(flit_d *in_flit);
	
	
	void UpdateVC(uint32_t rID,int vc,bool isfree);

	virtual void regStats();
	
  private:
  	
	//flitBuffer_d * getRecvBuffer(int routerId);
	//flitBuffer_d * getTargetBuffer(int destId);
    // ticks are only used for checking
    
	int num_vcs;
	int num_vcs_per_vnet;
	int inline get_vnet(int vc) {return vc/num_vcs_per_vnet;}
	
    //bool  Occupied;
    //Tick Occupied_start_Time;
    //int occupierId; // the router granted
    
    //int m_num_routers_in_group;
    
    std::vector<flitBuffer_d *> m_smart_router_in_buffers;
    std::vector<SmartInputUnit_d *> m_output_router_input_unit; // this is not allocated in this unit, simple a pointer for pointing the outport
    std::vector<Router_d *> m_router_list; // this is only used to fill inputUnit
    std::vector<SmartConfig *> route_compute_list;
	
    std::vector<uint32_t> router_seq;
	int router_id_to_index(uint32_t rID);
	uint32_t router_index_to_id(int index);
	
	
	// vc free information of each router
	std::vector<std::vector<bool> > ovc_free; //[# of router][# of vc]
	std::vector<std::vector<int> > vc_credits_for_smart_in_unit; // should be the buffer size 
	
	// here is for ovc arb
	int VCA_last_router_round;
	std::vector<int> VCA_last_inport_per_router; // [# of router]
	std::vector<int> VCA_last_vc_round_per_router; // [# of router]
	void VCAllocate();
		
	// here is for link arb
	int LA_last_router_round;
	std::vector<int> LA_last_vc_round_per_router;
	std::vector<int> LA_last_inport_per_router;
	void linkAllocte();
	
    //std:vector<bool> LA_router_requests;
	//std::vector<int> LA_current_select_vc_per_router;
	
	// routing use
	int getSmartConfigIndexOfDestRoute(int rID, int &iter_post);
	
	// send a flit
	void inline sendAFlit(Router_d * Cur_Router, int inport_id, int vc_id, int smart_buffer_index);
	
	//stats
	Stats::Scalar linkIdleCycle;
	Stats::Scalar linkWaitforVACycle;
	Stats::Vector FlitPerCycle;
	bool waiting_for_VA_grant();
	// helper functions
	
	void setOccupied( std::vector<bool> linkOccupied, int srcIndex, int destIndex );
	bool intersect( std::vector<bool> linkOccupied, int srcIndex, int destIndex);
};

#endif // __MEM_RUBY_NETWORK_GARNET_FIXED_PIPELINE_SMART_COORDINATE_HH__
