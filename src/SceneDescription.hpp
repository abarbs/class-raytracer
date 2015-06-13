#ifndef SCENE_H
#define SCENE_H

/**
   The description of a scene. Describes the light sources and objects in a 
   scene, as well as the collection of materials objects are composed of. 


   @author Andrew Barbarello
 */
#include <vector>
#include <stdexcept>
#include <Eigen/Dense>
#include <Eigen/StdVector>

#include "Light.hpp"
#include "Material.hpp"
#include "Camera.hpp"
#include "SceneObject.hpp"
#include "QuadricCollection.hpp"

class SceneDescription {
public:
    Camera cam;
    std::vector<std::unique_ptr<Light> > lights;
    std::vector<Material,Eigen::aligned_allocator<Material>> materials;
    std::vector<std::unique_ptr<SceneObject> > objects;
    void addLight(const Light &l) {
	if (static_cast<unsigned int> (l.id) != lights.size()) {
	    throw std::runtime_error("Encountered duplicate or out of order light id");
	}
	lights.emplace_back(new Light(l));
    }
    const std::vector<std::unique_ptr<Light> >& getLights() const {return lights;}
    /**
       Add a material for the scene. Once materials are added, they cannot
       be deleted. Materials are accessed by 
       {@ref SceneDescription::getMaterial(int id)} in the order they are 
       added. The first material added as {@code id} 0, the second has
       {@code id} 1, and so on.
     */
    void addMaterial(const Material &m) {materials.push_back(Material(m));}
    const Material& getMaterial(int id) const { return materials.at(id); }

    void setCamera(Camera &c) {cam = c;}
    
    void addObject(std::unique_ptr<SceneObject> o) {
	objects.emplace_back(std::move(o));
    }

    const std::vector<std::unique_ptr<SceneObject> >& getObjects() const {return objects;}
    
    virtual ~SceneDescription() {}


};

#endif
