/* Statistical Outlier Removal Filter has 2 main parameters
setMeanK() and setStddevMulThresh()

#include <string>
#include <boost/algorithm/string.hpp>

std::string value_string "0,0";
std::cout << "Values:" << '\n';
std::cin >> value_string;

boost::algorithm::split(value_array, value_string, is_any_of(","));

int meanK = std::stoi(value_array[0]);
int mulThresh = std::stoi(value_array[1]); // Error while using floats

s_filter.setMeanK(meanK);
s_filter.setStddevMulThresh(mulThresh);

*/
