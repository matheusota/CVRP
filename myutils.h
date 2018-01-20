// =============================================================
//
//  Some useful general routines
//
//  Send comments/corrections to Flavio K. Miyazawa.
//
// =============================================================
#ifndef MYUTILS_DEFINE
#define MYUTILS_DEFINE
#include <string>
#include <vector>
#include <time.h>
#include <math.h>
using namespace std;
//#include <ostream>
#include <sstream>
#include <iostream>  
#include <fstream>

//================================================================================================================
// SOME CONSTANTS
//
// maximum number of characters used by a command used by the system routine.
#define MAXCOMMANDSIZE 1000
// used to verify if number are the same, within some error of MY_EPS
#define MY_EPS 0.0000001
#define MY_INF 1000000000000.0

//================================================================================================================
//    ROUTINES TO DEAL WITH PDF
// set default pdf reader used in some programs
// For example, if it is possible to open a pdf file
// with the command "open file.pdf"
// then, you can use the following command to define
// the "open" as a pdf reader
// set_pdfreader("open");
void set_pdfreader(string programname);

// To see a pdf file. It uses the pdf reader defined by set_pdfreader.
int view_pdffile(string filename);


// Only to see if a file exists. It (tries to) open and close the file.
//
// bool FileExists(const char *filename)
// {
//   bool r;
//   ifstream ifile(filename);
//   r = ifile;
//   ifile.close();
//   return r;
// }
//
inline bool FileExists(const string& filename)
{
    ifstream f(filename.c_str());
    return f.is_open();
};




//====================================================================================================
//     * Type Conversion Routines
// In C++11 we have new convertion functions, but some libraries
// are still not compatible, like the Gurobi Library.

/* float              stof(const string& str, size_t *idx = 0); */
/* double             stod(const string& str, size_t *idx = 0); */
/* long double        stold(const string& str, size_t *idx = 0); */
/* int                stoi(const string& str, size_t *idx = 0, int base = 10); */
/* long               stol(const string& str, size_t *idx = 0, int base = 10); */
/* unsigned long      stoul(const string& str, size_t *idx = 0, int base = 10); */
/* long long          stoll(const string& str, size_t *idx = 0, int base = 10); */
/* unsigned long long stoull(const string& str, size_t *idx = 0, int base = 10); */
/* string to_string(int val); */
/* string to_string(unsigned val); */
/* string to_string(long val); */
/* string to_string(unsigned long val); */
/* string to_string(long long val); */
/* string to_string(unsigned long long val); */
/* string to_string(float val); */
/* string to_string(double val); */
/* string to_string(long double val); */

/* template <typename T> string ToString ( T val ){ */
/*     std::stringstream retval; */
/*     retval << val; */
/*     return retval; */
/* } */



// convert a double value to string
inline std::string DoubleToString(double val) 
{ std::stringstream out; out << val; return out.str(); } 
//{return to_string(val);} // this is only valid in c++11 


// convert a int value to string
inline std::string IntToString(int val)
{ std::stringstream out; out << val; return out.str(); }
//{return to_string(val);}  

// convert a string to double
inline double StringToDouble(string s)
{ double d;  stringstream(s) >> d;  return(d); }
//{return stod(s);} 

// convert a string to int
inline int StringToInt(string s)
{ int n;  stringstream(s) >> n;  return(n); }
//{return stoi(s);} 

inline void Pause(void) {cout<<"Pause";std::cin.get();cout<<"\n";}

//====================================================================================
//     * Functions to test values
// Return true if variable x is fractional (within a certain small error).
inline bool IsFrac(double x) {
  double f;
  f = ceil(x-MY_EPS)-x;
  if (f<MY_EPS) return(false);
  if (f>1.0-MY_EPS) return(false);
  return(true);  //    eps  <= ceil(x)-x <= 1-eps
}

// Return true if variable x is fractional (within a certain small error).
inline bool IsEqual(double x,double y) {
  if (x>y+MY_EPS) return(false);
  if (x<y-MY_EPS) return(false);
  return(true);  //    y-eps  <= x <= y+eps
}

// For the next functions, it is supposed that 0 <= X <= 1
inline bool BinaryIsOne(double x) { return((x > 1.0-MY_EPS)); }
inline bool BinaryIsZero(double x) { return((x < MY_EPS)); }
inline bool NonBinary(double x) { return((x > MY_EPS)&&(x<1.0-MY_EPS)); }

// verify if a vector contains only integer values
bool VectorIsInteger(vector<double> &v);


//====================================================================================
//     * Working with colors
// colors must match the ColorName in myutils.cpp
typedef enum Color {NOCOLOR,WHITE,BLACK,RED,GREEN,BLUE,YELLOW,MAGENTA,CYAN,GRAY,ORANGE} Color;

// Given a color code, return its name
std::string ColorName(int cor);

//================================================================================================================
//    ROUTINES FOR TIME MANIPULATION

long time70(void);  /* returns the time in seconds since Jan 1, 1970 */
void printtime(long t); /* print the time t in days, hours, minutes and seconds.*/
void sprinttime(char *s,long t); /* prints the time in the string s Example: 1 hour, 2 minutes, 3 seconds*/
void shortprinttime(long t); /* prints the time in the string s. Ex.: 11d,22h:33m:44s   */

#endif
