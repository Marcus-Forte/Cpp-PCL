#pragma once

#include <pcl/point_types.h>

namespace duna
{
    struct EIGEN_ALIGN16 _PointXYZINormalFeature
    {
        PCL_ADD_POINT4D;  // This adds the members x,y,z which can also be accessed using the point (which is float[4])
        PCL_ADD_NORMAL4D; // This adds the member normal[3] which can also be accessed using the point (which is float[4])
        union
        {
            struct
            {
                float intensity;
                float curvature;
            };
            float data_c[4];
        };
        PCL_MAKE_ALIGNED_OPERATOR_NEW
    };

    struct PointXYZINormalFeature : public _PointXYZINormalFeature
    {
        inline PointXYZINormalFeature(const _PointXYZINormalFeature &p)
        {
            x = p.x;
            y = p.y;
            z = p.z;
            data[3] = 1.0f;
            normal_x = p.normal_x;
            normal_y = p.normal_y;
            normal_z = p.normal_z;
            data_n[3] = 0.0f;
            curvature = p.curvature;
            intensity = p.intensity;
        }

        inline PointXYZINormalFeature()
        {
            x = y = z = 0.0f;
            data[3] = 1.0f;
            normal_x = normal_y = normal_z = data_n[3] = 0.0f;
            intensity = 0.0f;
            curvature = 0;
        }

        friend std::ostream &operator<<(std::ostream &os, const PointXYZINormalFeature &p);
    };

}