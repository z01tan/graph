#include <fstream>
#include <cstdlib>
#include <vector>
#include <boost/algorithm/string.hpp>
#include "graph.hpp"

int main(int argc, char** argv)
{
  std::fstream f;
  std::string line;
  f.open(/*"tsp_test.txt"*/argv[1]);
  std::vector<std::string> strs;
  getline(f,line);
  boost::split(strs,line,boost::is_any_of("\t "));
  int n = atoi(strs[0].c_str());
  Graph::Graph* g = new Graph::Graph();
  /*  while(getline(f,line))                                                             //FILL BY EDGES
    {
      boost::split(strs,line,boost::is_any_of("\t "));
      g->addEdge(atoi(strs[0].c_str())-1,atoi(strs[1].c_str())-1,atoi(strs[2].c_str()));

      }*/

  while(getline(f,line))
    {
      boost::split(strs,line,boost::is_any_of("\t "));
      g->addVertexEuclid(atoi(strs[0].c_str()) , atoi(strs[1].c_str()));
    }

  /*  int min;
  try{
    min = g->SSP();
    std::cout<<"ans is "<< min<<std::endl;
  }
  catch(Graph::Exception& e)
    {
      std::cout<<"exception: "<<e.what()<<std::endl;
    }
  
  */
  std::vector<std::pair<std::vector<Graph::Vertex*>,double>  >paths = g->TSPEuclid();//g->TSPEuclidRec(g->getVertex(1));
  int num = 1;

  std::pair<std::vector<Graph::Vertex*>, double> minimalpath = paths.front();
  

  for(std::vector<std::pair< std::vector <Graph::Vertex*>, double> >::iterator i = paths.begin(); i!=paths.end(); ++i)
    {
      //      std::cout<<"path number "<<num<<std::endl;
      /*for(std::vector<Graph::Vertex*>::iterator j = (std::get<0>(*i)).begin();j!=(std::get<0>(*i)).end();++j)
	{
	  std::cout<<"->"<<(*j)->getName();
	  }*/
      if((std::get<1>(*i)) < (std::get<1>(minimalpath)))
	minimalpath = (*i);
      num++;
      //      std::cout<<std::endl<<"lenght = "<<std::get<1>(*i)<<std::endl;
      //      std::cout<<std::endl<<std::endl;
	
    }
  for(std::vector<Graph::Vertex*>::iterator j = (std::get<0>(minimalpath)).begin();j!=(std::get<0>(minimalpath)).end();++j)
    {
      std::cout<<"->"<<(*j)->getName();
    }
  std::cout<<std::endl<<"minimal path cost = "<<std::get<1>(minimalpath)<<std::endl;
  f.close();
  return 0;
}
