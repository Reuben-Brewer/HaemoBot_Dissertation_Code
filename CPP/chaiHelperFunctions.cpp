#include "chaiHelperFunctions.h"

cVector3d convertDoubleVec3ToChai(std::vector<double> double_vec)
{
    cVector3d chai_vec;
    for(int i = 0; i <= 2; i++)
    {
        chai_vec[i] = double_vec[i];
    }
    return chai_vec;
}

void add_chai_model(vector<cMesh *> & model_vec, vector<string> & model_names, cWorld *world, string file_name, string model_name, double m_scale, double proxyRadius, double stiffness, cVector3d model_center, bool setGhost, cVector3d color, float transparency)

{
      model_vec.push_back(new cMesh(world));
      bool loadSuccess = model_vec.back()->loadFromFile(file_name);
      if(!loadSuccess)
      {
          printf("Model %s failed to load. \n", file_name.c_str());
      }
      else
      {
          printf("Model %s loaded properly. \n", file_name.c_str());
      }
      model_vec.back()->computeAllNormals(true);
      model_vec.back()->scale(cVector3d(m_scale, m_scale, m_scale), true);
      model_vec.back()->setPos(model_center);
      model_vec.back()->computeBoundaryBox(true);
      model_vec.back()->createAABBCollisionDetector(1.01 * proxyRadius, true, false);
      world->addChild(model_vec.back());
      model_names.push_back(model_name);
      model_vec.back()->setAsGhost(setGhost);
      model_vec.back()->setStiffness(stiffness, true);

      cMaterial my_material;
      my_material.m_ambient.set(color[0], color[1], color[2], transparency);
      my_material.setStiffness(stiffness);
      model_vec.back()->setMaterial(my_material, true);
      model_vec.back()->setUseMaterial(true, true);
      model_vec.back()->setUseVertexColors(false, true);
      model_vec.back()->setTransparencyRenderMode(true, true);
      model_vec.back()->setTransparencyLevel(transparency, true, true);
}
