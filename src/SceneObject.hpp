#ifndef SCENE_OBJECT_H
#define SCENE_OBJECT_H
#include <Eigen/Dense>
#include "Ray.hpp"
/**


   @author Andrew Barbarello
 */
class SceneObject {
public:
    int id,
	matId; /**< The identifier of the material of which this object 
		       is composed in the Vector of materials in 
		       SceneDescription */
    /**
       Return distance from origin of ray to nearest intersection with object
     */
    virtual double distToIntersection(const Ray &ray) = 0;
    /**
       Return normal vector to surface at the given point.
       @warning In production build, the method is not required to check
       that the given point lies on the surface of the object
     */
    virtual Eigen::Vector4d getUnitNormal(const Eigen::Vector4d &point) = 0;

    virtual ~SceneObject() {}

};
#endif
