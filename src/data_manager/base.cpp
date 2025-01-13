#include"data_manager/base.h"
#include"data_manager/param.h"


rm::ArmorColor Data::self_color;
rm::ArmorColor Data::mining_tank_color;
int Data::camera_index = 1;
std::vector<rm::Camera*> Data::camera;
bool Data::timeout_flag = true;