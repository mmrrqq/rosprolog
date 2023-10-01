
#include <string>
#include <iostream>

#include <rclcpp/rclcpp.hpp>
#include <rosprolog/rosprolog_client/PrologClient.h>

using namespace std;

int main(int argc, char *argv[])
{
	rclcpp::init(argc, argv);
	PrologClient pl;
	
	PrologQuery bdgs = pl.query("member(A, [1, 2, 3, 4]), B = ['x', A], C = foo(bar, A, B)");

	for(PrologQuery::iterator it=bdgs.begin();
	    it != bdgs.end(); it++)
	{
		PrologBindings bdg = *it;
		cout << "Found solution: " << endl;
		cout << "A = "<< bdg["A"] << endl;
		cout << "B = " << bdg["B"] << endl;
		cout << "C = " << bdg["C"] << endl;
	}
	return 0;
}
