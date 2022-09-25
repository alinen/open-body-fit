#include <map>

enum MoUnit {MM,CM,M,INCH,FT};  // Motion units
enum Weight {KG,LB}; // Weight units

static std::map<MoUnit, std::map<MoUnit,float>> initConversions()
{
    std::map<MoUnit, std::map<MoUnit,float>> m;
    m[MM][MM] = 1.0;    m[MM][CM] = 0.1;    m[MM][M] = 0.001;    m[MM][INCH] = 0.0393701;m[MM][FT] = 0.00328084;
    m[CM][MM] = 10.0;   m[CM][CM] = 1.0;    m[CM][M] = 0.01;     m[CM][INCH] = 0.393701; m[CM][FT] = 0.0328084;
    m[M][MM] = 1000.0;  m[M][CM] = 100.0;   m[M][M] = 1.0;       m[M][INCH] = 39.3701;   m[M][FT] = 3.28084;
    m[INCH][MM] = 25.4; m[INCH][CM] = 2.54; m[INCH][M] = 0.0254; m[INCH][INCH] = 1.0;    m[INCH][FT] = 0.0833333;
    m[FT][MM] = 304.8;  m[FT][CM] = 30.48;  m[FT][M] = 0.3048;   m[FT][INCH] = 12.0;     m[FT][FT] = 1.0;
    return m;
}

static std::map<Weight, std::map<Weight,float>> initWeightConversions()
{
    std::map<Weight, std::map<Weight,float>> m;
    m[LB][KG] = 0.453592;
    m[KG][LB] = 2.20462;
    return m;
}

std::map<MoUnit, std::map<MoUnit,float>> MoUnitTable = initConversions();
std::map<Weight, std::map<Weight,float>> WeightTable = initWeightConversions();


