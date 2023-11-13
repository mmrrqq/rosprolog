
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/executor.hpp>
#include <rclcpp/executors.hpp>
#include <console_bridge/console.h>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <signal.h>

#include <json_prolog_msgs/srv/prolog_next_solution.hpp>
#include <json_prolog_msgs/srv/prolog_query.hpp>
#include <json_prolog_msgs/srv/prolog_finish.hpp>

#include <rosprolog/rosprolog_node/PrologNode.h>

#define PARAM_INITIAL_PACKAGE "initial_package"
#define PARAM_INITIAL_GOAL "initial_goal"
#define PARAM_NUM_PL_THREADS "num_pl_threads"
#define PARAM_NUM_ROS_THREADS "num_ros_threads"
#define PARAM_ENABLE_DEBUG "pl_debug"

#define NUM_PL_THREADS_DEFAULT 2
#define NUM_ROS_THREADS_DEFAULT 2

PrologNode::PrologNode(const char* ns) : rclcpp::Node("prolog_node", ns),
	  is_initialized_(false),
	  thread_pool_(PrologNode::num_pl_threads(this))
{
	if(!ensure_loaded("rosprolog")) {
		return;
	}
	std::string param;
	// register initial packages
	if (get_parameter(PARAM_INITIAL_PACKAGE, param)) {
		if(!call1("register_ros_package",param)) {
			RCLCPP_ERROR(rclcpp::get_logger("rosprolog_node"),"Failed to load initial_package %s.", param.c_str());
			return;
		}
	}
	else if(!call1("register_ros_package","knowrob")) {
		RCLCPP_ERROR(rclcpp::get_logger("rosprolog_node"),"Failed to load knowrob.");
		return;
	}

	// execute initial goal
	{
		boost::shared_ptr<PrologEngine> engine = thread_pool_.claim();
		if (get_parameter(PARAM_INITIAL_GOAL, param)) {
			engine->one_solution(param);
			if(engine->has_error()) {
				RCLCPP_WARN(rclcpp::get_logger("rosprolog_node"),"initial goal failed: %s", engine->error().c_str());
			}
		}
		thread_pool_.release(engine);
	}

	// Create services
	prolog_query_ = create_service<json_prolog_msgs::srv::PrologQuery>(
		"/rosprolog/query", std::bind(&PrologNode::query, this, std::placeholders::_1, std::placeholders::_2));
	
	prolog_next_solution_ = create_service<json_prolog_msgs::srv::PrologNextSolution>(
		"/rosprolog/next_solution", std::bind(&PrologNode::next_solution, this, std::placeholders::_1, std::placeholders::_2));

	prolog_finish_ = create_service<json_prolog_msgs::srv::PrologFinish>(
		"/rosprolog/finish", std::bind(&PrologNode::finish, this, std::placeholders::_1, std::placeholders::_2));
		
	is_initialized_ = true;
}

int PrologNode::num_pl_threads(rclcpp::Node *node)
{
	int count = 0;
	if (node->get_parameter(PARAM_NUM_PL_THREADS, count)) {
		return count;
	}
	return NUM_PL_THREADS_DEFAULT;
}

int PrologNode::call1(const std::string &p, const std::string &arg1)
{
	term_t a1 = PL_new_term_refs(1); {
		PL_put_atom_chars(a1, arg1.c_str());
	}
	qid_t qid = PL_open_query(NULL, PL_Q_NODEBUG,
	    PL_pred(PL_new_functor(PL_new_atom(p.c_str()),1), NULL),
	    a1);
	bool status = TRUE;
	
	if(!PL_next_solution(qid)) {
		std::string error_msg;
		if(PrologEngine::pl_exception(qid,error_msg)) {
			RCLCPP_ERROR(rclcpp::get_logger("rosprolog_node"),"%s", error_msg.c_str());
		}
		status = FALSE;
	}
	// cleanup
	PL_close_query(qid);
	PL_reset_term_refs(a1);
	
	return status;
}

int PrologNode::ensure_loaded(const char *ros_pkg)
{
	std::stringstream ss;
	ss << ament_index_cpp::get_package_share_directory(ros_pkg) << "/src/__init__.pl";
	if(!call1("ensure_loaded",ss.str())) {
		RCLCPP_ERROR(rclcpp::get_logger("rosprolog_node"),"Failed to load __init__.pl of %s.", ros_pkg);
		return FALSE;
	}
	return TRUE;
}

bool PrologNode::exists(const std::string &id)
{
	return claimed_engines_.find(id) != claimed_engines_.end();
}

bool PrologNode::has_more_solutions(const std::string &id)
{
	return claimed_engines_.find(id)->second->has_more_solutions();
}

void PrologNode::finalize(const std::string &id)
{
	auto it = claimed_engines_.find(id);
	if(it != claimed_engines_.end()) {
		it->second->release(true);
		thread_pool_.release(it->second);
		claimed_engines_.erase(it);
	}
}

void PrologNode::finalize()
{
	auto it = claimed_engines_.begin();
	while (it != claimed_engines_.end()) {
		finalize(it->first);
		it = claimed_engines_.begin();
	}
}

bool PrologNode::query(const std::shared_ptr<json_prolog_msgs::srv::PrologQuery_Request> req,
			           std::shared_ptr<json_prolog_msgs::srv::PrologQuery_Response> res)
{
	if (exists(req->id)) {
		std::stringstream ss;
		ss << "Another query is already being processed with id " << req->id << ".";
		res->ok = false;
		res->message = ss.str();
	} else {
		boost::shared_ptr<PrologEngine> engine = thread_pool_.claim();
		engine->claim(req->query,
			(req->mode == json_prolog_msgs::srv::PrologQuery_Request::INCREMENTAL));
		claimed_engines_[req->id] = engine;
		res->ok = true;
		res->message = "";
	}
	return true;
}

bool PrologNode::finish(const std::shared_ptr<json_prolog_msgs::srv::PrologFinish_Request> req,
			            std::shared_ptr<json_prolog_msgs::srv::PrologFinish_Response> /*res*/)
{
	if (req->id == "*"){
		// finish all queries
		finalize();
	} else {
		finalize(req->id);
	}
	return true;
}

bool PrologNode::next_solution(const std::shared_ptr<json_prolog_msgs::srv::PrologNextSolution_Request> req,
				               std::shared_ptr<json_prolog_msgs::srv::PrologNextSolution_Response> res)
{
	if(!exists(req->id)) {
		res->status = json_prolog_msgs::srv::PrologNextSolution::Response::WRONG_ID;
		res->solution = "";
	}
	else {
		if (!has_more_solutions(req->id)){
			boost::shared_ptr<PrologEngine> x = claimed_engines_.find(req->id)->second;
			if(x->has_error()) {
				res->status = json_prolog_msgs::srv::PrologNextSolution::Response::QUERY_FAILED;
				res->solution = x->error();
				RCLCPP_WARN(rclcpp::get_logger("rosprolog_node"),"%s.", res->solution.c_str());
			}
			else {
				res->status = json_prolog_msgs::srv::PrologNextSolution::Response::NO_SOLUTION;
				res->solution = "";
			}
		}
		else {
			res->status = json_prolog_msgs::srv::PrologNextSolution::Response::OK;
			boost::shared_ptr<std::string> solution =
				claimed_engines_.find(req->id)->second->next_solution();
			res->solution = solution->c_str();
		}
	}
	return true;
}

// Signal-safe flag for whether shutdown is requested
sig_atomic_t volatile g_request_shutdown = 0;
void sigint_handler(int /*sig*/)
{
	g_request_shutdown=1;
}

int main(int argc, char **argv)
{
	rclcpp::init(argc, argv);	
	// loop at 10Hz
	rclcpp::Rate loop_rate(10.0); //Hz
	// executor.spin();
	// initialize Prolog
	int pl_ac = 0;
	char *pl_av[5]; {
		pl_av[pl_ac++] = argv[0];
		// '-g true' is used to suppress the welcome message
		pl_av[pl_ac++] = (char *) "-g";
		pl_av[pl_ac++] = (char *) "true";
		// Inhibit any signal handling by Prolog
		// TODO use version macro -nosignals needed for old version
		// pl_av[pl_ac++] = (char *) "-nosignals";
		pl_av[pl_ac++] = (char *) "--signals=false";
		// Limit the combined size of the Prolog stacks to the indicated size.
		//pl_av[pl_ac++] = (char *) "--stack_limit=32g";
		// Limit for the table space.
		// This is where tries holding memoized11 answers for tabling are stored.
		//pl_av[pl_ac++] = (char *) "--table_space=32g";
		//pl_av[pl_ac++] = (char *) "-G256M";
		pl_av[pl_ac] = NULL;
	}
	// NOTE: PL init must be called before creating the node!
	PL_initialise(pl_ac, pl_av);

	auto rosprolog = std::make_shared<PrologNode>("rosprolog");
	// rosprolog can serve requests in parallel
	int num_ros_threads;
	if(!rosprolog->get_parameter(PARAM_NUM_ROS_THREADS, num_ros_threads)) {
		num_ros_threads = NUM_ROS_THREADS_DEFAULT;
	}
	rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(), num_ros_threads);
	executor.add_node(rosprolog);
	// Override the default ros sigint handler.
	// This must be set after the first NodeHandle is created.
	signal(SIGINT, sigint_handler);
	//
	if(rosprolog->is_initialized()) {
		RCLCPP_INFO(rclcpp::get_logger("rosprolog"),"rosprolog service is running.");		
		while (rclcpp::ok() && !g_request_shutdown) {
			executor.spin_some();
			loop_rate.sleep();
		}
		RCLCPP_INFO(rclcpp::get_logger("rosprolog"),"rosprolog service is exiting.");
		rosprolog->finalize();
  		rclcpp::shutdown();
		
		return EXIT_SUCCESS;
	}
	else {
		RCLCPP_ERROR(rclcpp::get_logger("rosprolog"),"rosprolog service failed to initialize.");
		return EXIT_FAILURE;
	}
}
