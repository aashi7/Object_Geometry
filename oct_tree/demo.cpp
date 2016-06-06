#include "octree.h"
#include <iostream>
#include <cstring>

#ifdef WIN32
#include <tchar.h>
int _tmain(int argc, _TCHAR* argv[])
#else
int main(int argc, char* argv[])
#endif
{
	Octree<double> o(4096); /* Create 4096x4096x4096 octree containing doubles. */
	
	//std::cout<<o.nodes()<<std::endl;	
	o(1,1,1) = 5;
	o(1,1,2) = 5;
	Node* root= o.root();
	o(1,2,2) = 5;
	o(10,10,10) = 5;
	o(11,10,10) = 5;
	o.set(1,1,1, 1);
	std::cout<<"(1,1,1) = "<<o.at(1,1,1)<<std::endl;
	//o(1,1,1) = 3.1416;      /* Put pi in (1,2,3). */
	std::cout<<"size of octree: "<<o.size()<<std::endl;
	std::cout<<"number of nodes instantiated in tree: "<<o.nodes()<<std::endl;
	//std::cout<<o.at(1,2,3)<<std::endl;
	o.erase(1,2,3);         /* Erase that node. */
	o.erase(1,1,2);         /* Erase that node. */
	o.erase(1,2,2);         /* Erase that node. */
	o.erase(10,10,10);         /* Erase that node. */

	return 0;
}

