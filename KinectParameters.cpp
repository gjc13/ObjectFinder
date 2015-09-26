//
// Created by 郭嘉丞 on 15/9/25.
//

#include "KinectParameters.h"

double depthToZ[2] = {446.50072208, -3.81086495};
double projectionParameter[3][4] = {
        {-3.24164382e-02, -1.04860576e-03, 2.32010121e-02, 9.97794984e-01},
        {-2.18961293e-04, -3.30200458e-02, 1.99508316e-02, 3.64211223e-02},
        {-1.11424893e-06, -3.74821199e-06, 9.04515172e-05, 2.12172341e-04}
};

double projectionParameter1080[3][4] = {
        {-1.27663807e-02, -1.19225174e-02, 6.65230093e-03, 9.58467014e-01},
        {-1.49256898e-03, -1.55655743e-02, 4.43661057e-03, 2.84125098e-01},
        {-2.84752072e-06, -1.12286641e-05, 7.11675446e-06, 5.77375615e-04}
};