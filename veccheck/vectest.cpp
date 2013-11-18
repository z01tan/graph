#include <vector>
#include <iostream>

std::vector<std::vector <int> > testFun()
{
  std::vector<std::vector<int> > res;
  
  return res;
  
}


int main()
{
  std::vector <std::vector <int> >  iv = testFun();
  std::vector<int> p;
  p.push_back(22);
  iv.push_back(p);
  std::cout<<"pzpz"<<std::endl;
  
  iv[0].push_back(23);
  std::cout<<"pzpz"<<std::endl;
  for(std::vector<std::vector <int> >::iterator it = iv.begin();it!=iv.end();++it)
    {
      for(std::vector<int>::iterator jt = (*it).begin() ; jt!=(*it).end(); ++jt)
	{
	  std::cout<<"iv = "<<(*jt)<<std::endl;
	}
    }
  return 0;
}
