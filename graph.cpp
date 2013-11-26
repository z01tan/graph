#include "graph.hpp"
#include <algorithm>
#include <iostream>
#include <limits>
#include <exception>

#include <cmath>
//#include <utility>


namespace Graph
{
  //Edge
  Edge::Edge()
  {
    
  }

  Edge::Edge(Vertex* from, Vertex* to, double w)
  {
    From   = from;
    To     = to;
    Weight = w;
  }

  Edge::~Edge()
  {
    
  }
  
  double Edge::getWeight() const
  {
    return Weight;
  }
  
  Vertex* Edge::getFrom()
  {
    return From;
  }

  Vertex* Edge::getTo()
  {
    return To;
  }

  void Edge::setTo(Vertex* t)
  {
    To = t;
  }

  void Edge::setFrom(Vertex* f)
  {
    From = f;
  }

  void Edge::setWeight(int w)
  {
    Weight = w;
  }

  //Vertex
  Vertex::Vertex()
  {
    parentPointer = this;
    Key = 1;
  }

  Vertex::Vertex(int n)
  {
    Name = n;
    parentPointer = this;
    Key = 1;
  }
  
  Vertex::~Vertex()
  {}
  
  int Vertex::getName()
  {
    return Name;
  }

  
  void Vertex::setName(int n)
  {
    Name=n;
  }

  int Vertex::getKey()
  {
    return Key;
  }

  void Vertex::setKey(int k)
  {
    Key=k;
  }

  Vertex* Vertex::getParentPointer()
  {
    return parentPointer;
  }

  void Vertex::setParentPointer(Vertex* v)
  {
    parentPointer = v;
  }

  Vertex* Vertex::getRoot()
  {
    if(parentPointer!=this)
      {
	return parentPointer->getRoot();
      }
    else
      {
	return this;
      }
  }

  std::vector<Edge*> Vertex::getEdges()
  {
    return edges;
  }
  
  void Vertex::setEdges(std::vector <Edge*> l)
  {
    edges = l;
  }

  void Vertex::addEdge(Edge* l)
  {
    edges.push_back(l);
  }

  //EuclidVertex

  void EuclidVertex::setXCoord(float x)
  {
    xcoord = x;
  }

  void EuclidVertex::setYCoord(float y)
  {
    ycoord = y;
  }

  float EuclidVertex::getXCoord()
  {
    return xcoord;
  }

  float EuclidVertex::getYCoord()
  {
    return ycoord;
  }
  //Graph

  bool Compare(const Edge* a, const Edge* b)
  {
    return a->getWeight() > b->getWeight();
  }

  Graph::Graph()
  {
  }

  Graph::Graph(int n)
  {
    for(int i=0;i<n;++i)
      {
	v.push_back(new Vertex(i+1));
	
      }

#ifdef HEAP
    std::make_heap(e.begin(),e.end(),Compare);
#endif

  }

  Graph::~Graph()
  {}

  void Graph::addEdge(int from, int to, double cost)
  {
    Edge* ed = new Edge(this->getVertex(from),this->getVertex(to),cost);
    this->getVertex(from)->addEdge(ed);
    this->getVertex(to)->addEdge(ed);
    e.push_back(ed);
#ifdef HEAP
    std::push_heap(e.begin(),e.end(),Compare);
#endif
   

  }
  

  void Graph::addVertexEuclid(float xcoord, float ycoord)
  {
    EuclidVertex* ver = new EuclidVertex(xcoord, ycoord, v.size()+1);
    float x1,y1,x2,y2;
    for(std::vector<Vertex*>::iterator it = v.begin(); it!=v.end();++it)
      {

	x1 = ver->getXCoord();
	y1 = ver->getYCoord();
	x2 = ((EuclidVertex*)(*it))->getXCoord();
	y2 = ((EuclidVertex*)(*it))->getYCoord();
	Edge* ed = new Edge((Vertex*)ver,(*it), sqrt(pow(x1 - x2, 2)+pow(y1 - y2,2)));
	ver->addEdge(ed);
	(*it)->addEdge(ed);
	e.push_back(ed);
	
      }
    v.push_back(ver);
  }

  Vertex* Graph::getVertex(int n)
  {
    //    std::cout<<"getVertex"<<std::endl;
    return v[n];
  }

  std::vector<Vertex*>* Graph::getVerList()
  {
    return &v;
  }

  void Graph::Clustering(int n)
  {
    
    int count=0;
    int k = v.size();
    std::cout<<"ke="<<k<<std::endl;
    while(k>n)
      {
	Edge*   edge  = e.front();
	Vertex* from  = edge->getFrom();
	Vertex* to    = edge->getTo();
	Vertex* fRoot = from->getRoot();
	Vertex* tRoot = to->getRoot();
	if(fRoot!=tRoot){
#ifdef DEBUG
	  std::cout<<from->getName()<<"<-->"<<to->getName()<<std::endl;
#endif
	  if(fRoot->getKey() >= tRoot->getKey())
	    {
	      fRoot->setKey(fRoot->getKey() + tRoot->getKey());
	      tRoot->setParentPointer(fRoot);
	    }
	  else
	    {
	      tRoot->setKey(fRoot->getKey() + tRoot->getKey());
	      fRoot->setParentPointer(tRoot);
	    }
	  ++count;
	  --k;
	}
#ifdef HEAP	
	std::pop_heap(e.begin(),e.end(),Compare);
#endif
	//	std::cout << e.back()->getWeight()<<std::endl;
	e.pop_back();


      }

    while(e.front()->getFrom()->getRoot() == e.front()->getTo()->getRoot())
      {
#ifdef DEBUG
	std::cout<<e.front()->getFrom()->getName()<<" "<<e.front()->getTo()->getName()<<" "<<e.front()->getWeight()<<std::endl;
#endif
#ifdef HEAP
	std::pop_heap(e.begin(),e.end(),Compare);
#endif
	e.pop_back();
      }

    std::cout<<"spacing = "<<e.front()->getWeight()<<std::endl;
    std::cout<<k<<std::endl;
    
  }

  int Graph::BellFo(int s)

  {
    int V = v.size();
    float** A = new float*[V];
    for(int i=0; i<V;++i)
      A[i] = new float[V];

    float inf = std::numeric_limits<float>::infinity(); 
	
    for(int ver=0;ver<V;++ver)
      {
	for(int i=0;i<V;++i)
	  {
	    A[ver][i]=inf;
	  }
      }

    for(int ind=0;ind<V;++ind)
      A[s-1][ind]=0;

    int count;
    for(int i=1;i<V;++i)
      {
	count = 0 ;
	for(std::vector<Edge*>::iterator eit = e.begin();eit!=e.end();++eit)
	  {
	    int u = (*eit)->getFrom()->getName();
	    int v = (*eit)->getTo()->getName();
	    int w = (*eit)->getWeight();

	    if(A[v-1][i] > A[u-1][i-1] + w)
	      {

		for(int z = i; z<V; ++z)
		  A[v-1][z] = A[u-1][i-1]+w;
		
	      }
	    count++;
	  }
      }
    
    bool hasNegativeCycle = false;
    for(int ind=0;ind<V;ind++)
      hasNegativeCycle = hasNegativeCycle or (A[ind][V-2]!=A[ind][V-1]);
    
    if(hasNegativeCycle){
      throw Exception();
    }
#ifdef DEBUG    
    else
      std::cout<<"No negative cyle detected"<<std::endl;
#endif    
    int min = A[0][V-1];

    for(int ind=0;ind<V;ind++){
#ifdef DEBUG
      std::cout<<"vertex["<<ind+1<<"] "<<A[ind][V-1]<<std::endl; 
#endif
      if(min>A[ind][V-1])
	min = A[ind][V - 1];
    }
#ifdef DEBUG
    std::cout<<std::endl<<std::endl<<"shortest shortest path "<<min<<std::endl;
#endif
    for(int i=0;i<V;++i)
      delete[] A[i];
    delete[] A;
    return min;
  }

  int Graph::SSP()
  {
    int min=BellFo(v[0]->getName());

    for(int i=1;i<v.size();++i)
      {
	int lm = BellFo(v[i]->getName());
	if(min>lm)
	  min = lm;
      }

    return min;
  }

  std::vector<std::pair< std::vector<Vertex*>, double> > Graph::TSPEuclidRec(Vertex* curVer)
  {
    std::vector<Edge*> edges = curVer->getEdges();
    std::vector< std::pair<std::vector<Vertex*>, double> > pathacc;
    int tmpKey = curVer->getKey();
    curVer->setKey(0);
    for(std::vector< Edge* >::iterator i= edges.begin(); i!=edges.end();++i)
      {
	Vertex* tmpVer = (*i)->getTo();

	if(tmpVer==curVer)
	  tmpVer = (*i)->getFrom();
	  
	if(tmpVer->getKey()!=0)
	  {
	    std::vector<std::pair< std::vector<Vertex *> , double > > subTsp =  TSPEuclidRec(tmpVer);
	    for(std::vector<std::pair<std::vector<Vertex *> , double> >::iterator j = subTsp.begin(); j!=subTsp.end();++j)
	      {

		(std::get<0>(*j)).insert((std::get<0>(*j)).begin(),curVer);
		std::get<1>(*j) += (*i)->getWeight();
		pathacc.push_back(*j);

	      }
	    tmpVer->setKey(1);
	  }
      }
    curVer->setKey(tmpKey);
    if(!pathacc.empty())
      return pathacc;
    else
      {
	std::vector<std::pair<std::vector<Vertex*>, double> >res;
	std::vector<Vertex*> p;
	p.push_back(curVer);
	res.push_back(std::pair<std::vector<Vertex*>, double>(p,0));
	return res;

      }
  }

  std::vector<std::pair< std::vector<Vertex*>, double> > Graph::TSPEuclidIter(Vertex* curVer)
  {
    std::vector< std::pair<std::vector<Vertex*>, double> > pathacc;
    
    return pathacc;
  }
  
  std::vector<std::pair< std::vector<Vertex*>, double> > Graph::TSPHeld_Karp()
  {
    std::vector< std::pair<std::vector<Vertex*>, double> > pathacc;
    
    return pathacc;
    
  }

  std::vector<std::pair<std::vector<Vertex*>, double> > Graph::TSPEuclid()
  {
    std::vector<std::pair<std::vector<Vertex*>, double> > acycle = TSPEuclidRec(getVertex(1));
    float x1,x2,y1,y2;
    for(std::vector<std::pair<std::vector<Vertex*>, double > >::iterator it = acycle.begin(); it != acycle.end(); ++it)
      {

	x1 = ((EuclidVertex*)(std::get<0>(*it)).front())->getXCoord();
	x2 = ((EuclidVertex*)(std::get<0>(*it)).back())->getXCoord();
	y1 = ((EuclidVertex*)(std::get<0>(*it)).front())->getYCoord();
	y2 = ((EuclidVertex*)(std::get<0>(*it)).back())->getYCoord();
	std::get<1>(*it)+=sqrt(pow(x2 - x1,2)+pow(y2 - y1,2));
      }
    return acycle;
  }

}
