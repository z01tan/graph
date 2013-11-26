#ifndef GRAPH
#define GRAPH

//#define DEBUG

// #define HEAP

#include <vector>

namespace Graph
{
  class Vertex;

  class Edge
  {
    double Weight;
    Vertex* From;
    Vertex* To;
  public:
    Edge();
    Edge(Vertex*,Vertex*,double);
    ~Edge();
    double getWeight() const;
    Vertex* getFrom();
    Vertex* getTo();
    void setWeight(int);
    void setFrom(Vertex*);
    void setTo(Vertex*);

  };
  
  class Vertex
  {
    int Name;
    int Key;
    Vertex* parentPointer;
    std::vector<Edge*> edges;
  public:
    Vertex();
    Vertex(int);
    ~Vertex();
    int getName();
    void setName(int);
    int getKey();
    void setKey(int);
    Vertex* getParentPointer();
    Vertex* getRoot();
    void setParentPointer(Vertex*);
    std::vector<Edge*> getEdges();
    void setEdges(std::vector<Edge*>);
    void addEdge(Edge*);
      
  };

  class EuclidVertex: public Vertex
  {
    float xcoord;
    float ycoord;
    
  public:
    EuclidVertex():Vertex(),xcoord(0),ycoord(0)
    {};
    EuclidVertex(int n):Vertex(n),xcoord(0),ycoord(0)
    {};
    EuclidVertex(float x, float y, int n):Vertex(n),xcoord(x),ycoord(y)
    {};
    void setXCoord(float xcoord);
    float getXCoord();
    void setYCoord(float ycoord);
    float getYCoord();
    ~EuclidVertex();
    
  };

  class Graph
  {
    std::vector<Edge*> e;
    std::vector<Vertex*> v;
  public:
    Graph();
    Graph(int);
    ~Graph();
    void addEdge(int, int, double);
    void addVertexEuclid(float, float);
    Vertex* getVertex(int);
    std::vector<Vertex*>* getVerList();
    void Clustering(int);
    int BellFo(int);
    int SSP();
    std::vector<std::pair< std::vector<Vertex*>, double> > TSPEuclidRec(Vertex*);
    std::vector<std::pair< std::vector<Vertex*>, double> > TSPEuclidIter(Vertex*);
    std::vector<std::pair< std::vector<Vertex*>, double> > TSPHeld_Karp();
    std::vector<std::pair< std::vector<Vertex*>, double> > TSPEuclid();

    friend bool Compare(const Edge*, const Edge*);
   
  };
  
  class Exception : public std::exception{
  public:
    enum ExceptionType
      {
	NEGATIVE_CYCLE
      };
    
    virtual ExceptionType getExceptionType() const throw()
    {
      return NEGATIVE_CYCLE;
    }
    
    virtual const char* what() throw() {return "NETATIVE_CYCLE"; }
  };


}

#endif
