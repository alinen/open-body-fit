#ifndef UnitUtils_H_
#define UnitUtils_H_

#include <map>

enum MoUnit {MM,CM,M,INCH,FT};  // Motion units
enum Weight {KG,LB}; // Weight units

extern std::map<MoUnit, std::map<MoUnit,float>> MoUnitTable;
extern std::map<Weight, std::map<Weight,float>> WeightTable;

#endif
