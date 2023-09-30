/* 
 * Copyright (c) 2010, Lorenz Moesenlechner <moesenle@cs.tum.edu>
 * All rights reserved.
 * 
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 * 
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of Willow Garage, Inc. nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 * 
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#include <boost/lexical_cast.hpp>

#include <rclcpp/rclcpp.hpp>

#include <rosprolog/rosprolog_client/PrologQuery.h>
#include <rosprolog/rosprolog_client/PrologClient.h>

#include <json_prolog_msgs/srv/prolog_query.hpp>
#include <json_prolog_msgs/srv/prolog_next_solution.hpp>
#include <json_prolog_msgs/srv/prolog_finish.hpp>

void PrologQuery::iterator::increment()
{
	if(!query_ || data_ == query_->bindings_.end())
	{
		throw PrologQuery::QueryError("Cannot increment a query end iterator.");
	}
	{
		std::list<PrologBindings>::iterator it = data_;
		if(++it != query_->bindings_.end())
		{
		    ++data_;
		    return;
		}
	}
	if(query_->finished_)
	{
		data_ = query_->bindings_.end();
		return;
	}
	if(requestNextSolution())
	{
		++data_;
	}
}

bool PrologQuery::iterator::requestNextSolution()
{
	auto req = std::make_shared<json_prolog_msgs::srv::PrologNextSolution_Request>();

	req->id = query_->query_id_;
	auto result = query_->prolog_->next_solution->async_send_request(req);
  	if (rclcpp::spin_until_future_complete(query_->prolog_->get_node_base_interface(), result) != rclcpp::FutureReturnCode::SUCCESS){
		throw PrologQuery::QueryError("Service call failed.");
  	}
	
	switch(result.get()->status){
	case json_prolog_msgs::srv::PrologNextSolution_Response::NO_SOLUTION:
		data_ = query_->bindings_.end();
			query_->finished_ = true;
		return false;
	case json_prolog_msgs::srv::PrologNextSolution_Response::WRONG_ID:
		query_->finished_ = true;
		throw PrologQuery::QueryError(
		    "Wrong id. Maybe the server is already processing a query.");
	case json_prolog_msgs::srv::PrologNextSolution_Response::QUERY_FAILED:
		query_->finished_ = true;      
		throw PrologQuery::QueryError(
		    "Prolog query failed: " + result.get()->solution);
	case json_prolog_msgs::srv::PrologNextSolution_Response::OK:
		query_->bindings_.push_back(PrologBindings::parseJSONBindings(result.get()->solution));
		return true;
	default:
		query_->finished_ = true;      
		throw PrologQuery::QueryError("Unknow query status.");
	}
}

bool PrologQuery::iterator::equal(const iterator &other) const
{
	if(!other.query_ && query_ && data_ == query_->bindings_.end())
		return true;
	else if(!query_ && other.query_ && other.data_ == other.query_->bindings_.end())
		return true;
	else
		return (!query_ && !other.query_) || (data_ == other.data_);
}

PrologQuery::PrologQuery(PrologClient &prolog, const std::string &query_str)
: finished_(false),
  prolog_(&prolog),
  query_id_(makeQueryId())
{
	auto req = std::make_shared<json_prolog_msgs::srv::PrologQuery_Request>();

	req->id = query_id_;
	req->query = query_str;

	if(!prolog_->prolog_query->get_client_handle() || !prolog_->prolog_query->service_is_ready())
	{
		throw ServerNotFound("No connection to the rosprolog server.");
	}

	auto result = prolog_->prolog_query->async_send_request(req);
  	if (rclcpp::spin_until_future_complete(prolog_->get_node_base_interface(), result) != rclcpp::FutureReturnCode::SUCCESS){
		throw PrologQuery::QueryError(std::string("Service call '") + prolog_->prolog_query->get_service_name() + "' failed");
  	}

	if(!result.get()->ok)
	{
		throw PrologQuery::QueryError("Prolog query failed: " + result.get()->message);
	}
	// Instantiate the first solution
	PrologQuery::iterator(*this).requestNextSolution();
}

PrologQuery::iterator PrologQuery::begin()
{
	return PrologQuery::iterator(*this);
}

void PrologQuery::finish()
{
	auto req = std::make_shared<json_prolog_msgs::srv::PrologFinish_Request>();
	
	req->id = query_id_;
	auto result = prolog_->prolog_finish->async_send_request(req);
  	if (rclcpp::spin_until_future_complete(prolog_->get_node_base_interface(), result) != rclcpp::FutureReturnCode::SUCCESS){
		throw PrologQuery::QueryError(std::string("Service call '") + prolog_->prolog_finish->get_service_name() + "' failed");
  	}
	finished_ = true;
}

std::string PrologQuery::makeQueryId()
{
	static int counter = 0;
	rclcpp::Clock system_clock(RCL_SYSTEM_TIME);
	const rclcpp::Time& time = system_clock.now();
	return "ROSPROLOG_CPP_" + boost::lexical_cast<std::string>(time.nanoseconds()) +
		boost::lexical_cast<std::string>(counter++);
}
