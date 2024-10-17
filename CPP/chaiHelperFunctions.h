#ifndef CHAI_HELPER_FUNCTIONS_H
#define CHAI_HELPER_FUNCTIONS_H

#include <string>
#include <vector>
#include "chai3d.h"

void add_chai_model(vector<cMesh *> & model_vec, vector<string> & model_names, cWorld *world, string file_name, string model_name, double m_scale, double proxyRadius, double stiffness, cVector3d model_center, bool setGhost, cVector3d color, float transparency);
cVector3d convertDoubleVec3ToChai(std::vector<double> double_vec);

#endif
