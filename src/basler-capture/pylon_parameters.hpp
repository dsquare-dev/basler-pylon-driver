//
// Created by anyone on 23/02/17.
//

#ifndef KIWIBOT_SCHEDULER_PARAMETERS_H
#define KIWIBOT_SCHEDULER_PARAMETERS_H

#include <kiwibot_lib/kiwibot_parameter_v2.h>
#include <kiwibot_lib/template_singleton.h>
#include <ros/package.h>
#include <fstream>

namespace kiwibot {

    namespace pylon{
      const std::string LEFT_CAMERA_S  = "left_camera";
      const std::string RIGHT_CAMERA_S = "right_camera";
    }

    class PylonParameters : public KiwibotParameters, public TemplateSingleton<PylonParameters>{
        friend class TemplateSingleton<PylonParameters>;//Magic allows TemplateSingleton to use private constructor

    public:

    private:
        PylonParameters(){
          addParameter(pylon::LEFT_CAMERA_S, std::string("left"));
          addParameter(pylon::RIGHT_CAMERA_S, std::string("right"));
        }
    };
}

#endif //KIWIBOT_SCHEDULER_PARAMETERS_H
