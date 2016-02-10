
#include <iostream>
#include <fstream>
#include <string>
#include <sstream>
#include <vector>
#include <stdlib.h>

std::string outfilename("model.txt");

void getTokens(std::string& line,std::vector<float> & tokens){
  int tokencount = 0;
  std::stringstream res;
  for(int i = 0;i<line.length();i++){
      if(tokencount<3){
        if(line[i] != ' ')
        {
          res<<line[i];
        }else{
          std::string result = res.str();
          tokens.push_back(atof(result.c_str()));
          tokencount++;
          res.str(std::string());
        }
      }else
        return;
  }
}  

int main (int argc,char** argv) {
  
  
  
  if(argc<2){
    std::cout<<"please specify an input filename containing the model scanned from skanect"<<std::endl;
    return 0;
  }
  if(argc == 3){
    outfilename = std::string(argv[2]) + ".txt";
  }

  std::ofstream ofs;
  ofs.open(outfilename.c_str(),std::ofstream::out | std::ofstream::trunc);
  std::string line;
  
  std::ifstream myfile (argv[1]);
  int linecount = 0;
  if (myfile.is_open())
  {
    while ( getline (myfile,line) )
    {
      if(linecount >= 13){
          std::vector<float> tokens;
          getTokens(line,tokens);
          ofs<<tokens[0]<<" "<<tokens[1]<<" "<<tokens[2]<<std::endl;
          linecount++;
      }else
        linecount++;
    }
    myfile.close();
  }
  else 
    std::cout << "Unable to open file"<<argv[1];

  std::cout<<"succesfully converted "<<linecount-14<<"lines"<<std::endl;   
  
  myfile.close();
  ofs.close();
  return 0;
}