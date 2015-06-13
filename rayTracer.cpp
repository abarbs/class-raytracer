#include <iostream>
#include <fstream>
#include <sstream>
#include <string>
#include <algorithm>
#include <limits>

#include <boost/algorithm/string/trim.hpp>
#include <Eigen/StdVector>

#include "SceneDescription.hpp"
#include "Camera.hpp"
#include "Sphere.hpp"
#include "Ellipsoid.hpp"
#include "Cylinder.hpp"
#include "Box.hpp"
#include "Ray.hpp"
#include "consts.hpp"

typedef struct intersection {
    int objId;
    double dist;
} intersection_t;

void parseModelFile(std::istream& input, SceneDescription& scene);
std::unique_ptr<unsigned char[]> getRaster();
void writePPM(std::unique_ptr<unsigned char[]> buf, unsigned int rows, unsigned int cols, std::string filename);
Eigen::Vector4d getColor(const Ray&, intersection_t, unsigned int recursionLevel, unsigned int transDepth);
intersection_t getIntersection(const Ray&);
bool refract;
bool reflect;
bool aa;
SceneDescription scene;

#ifndef _UNIT_TESTS
int main(int argc, char **argv) {
    if (argc < 6) {
	std::cerr <<  "Usage: raytracer <model_file> <output_file_name> <refract> <reflect> <aa>" << std::endl;
	exit(1);
    }
    std::ifstream input(argv[1]);
    parseModelFile(input, scene);
    std::string fname = argv[2];
    refract = (boost::starts_with(argv[3], "t")) ? true : false;
    reflect = (boost::starts_with(argv[4], "t")) ? true : false;
    aa = (boost::starts_with(argv[5], "t")) ? true : false;
    auto buf = getRaster();
    writePPM(std::move(buf), scene.cam.viewportHeight, scene.cam.viewportWidth, argv[2]);
    return 0;
}
#endif

/**
   Add objects described in input to scene.
   @param[in] input The input stream to read scene description from.
   @param[out] scene The {@ref SceneDescription scene} to add the objects described in {@code input} to
*/
void parseModelFile(std::istream& input, SceneDescription& scene) {
    std::string line;
    while (std::getline(input, line)) {
	boost::trim(line);
	try {
            if (boost::starts_with(line, "#")) {
                continue;
            } else if (boost::starts_with(line, "light")) {
		scene.addLight(Light::fromString(line));
	    } else if (boost::starts_with(line, "mat")) {
		scene.addMaterial(Material::fromString(line));
	    } else if (boost::starts_with(line, "obj")) {
		if (line.find("sphere") != std::string::npos) {
		    Sphere s = Sphere::fromString(line);
		    scene.addObject(std::unique_ptr<Sphere>(new Sphere(s)));
		} else if (line.find("ellipsoid") != std::string::npos) {
		    Ellipsoid e = Ellipsoid::fromString(line);
		    scene.addObject(std::unique_ptr<Ellipsoid>(new Ellipsoid(e)));
		} else if (line.find("box") != std::string::npos) {
		    Box b = Box::fromString(line);
		    scene.addObject(std::unique_ptr<Box>(new Box(b)));
		} else if (line.find("cylinder") != std::string::npos) {
		    Cylinder c = Cylinder::fromString(line);
		    scene.addObject(std::unique_ptr<Cylinder>(new Cylinder(c)));
		} else {
		    std::cerr << "Unrecognized object type " << line << std::endl;
		}
	    } else if (boost::starts_with(line, "camera")
		       || boost::starts_with(line, "resolution")
		       || boost::starts_with(line, "viewport")) {
		std::stringstream cameraLines;
		// Get the remaining 2 "camera-related" lines
		cameraLines << line;
		for (int i = 0; i < 2; ++i) {
		    do {
			std::getline(input, line);
			boost::trim(line);
		    } while (line.length() == 0);
		    cameraLines << line;
		}
		Camera c = Camera::fromString(cameraLines.str());
		scene.setCamera(c);
	    }
	    else if (line.length() == 0) {
		continue;
	    } else {
		std::cerr << "Couldn't recognize first token of line: "
			  << line << std::endl;
	    }
	} catch (const std::string &str) {
	    std::cout << str << std::endl;
	} catch (const char *cstr) {
	    std::cout << cstr << std::endl;
	} catch (std::exception& e) {
	    std::cout << e.what() << std::endl;
	}
    }
}

intersection_t getIntersection(const Ray &ray) {
    const std::vector<std::unique_ptr<SceneObject> > &objects = scene.getObjects();
    double minDistance = std::numeric_limits<double>::infinity();
    int objId = -1;

    for (unsigned int i = 0; i < objects.size(); ++i) {
        double dist = objects[i]->distToIntersection(ray);
        if (dist  < minDistance && (dist > EPSILON || i != ray.fromObj)) {
            minDistance = dist;
            objId = i;
        }
    }
    intersection_t result = {objId, minDistance};
    return result;
}

Eigen::Vector4d getColor(const Ray &ray, unsigned int recursionLevel, unsigned int transDepth) {
    BOOST_ASSERT_MSG(std::abs(1 - ray.dir.norm()) < EPSILON, "Got ray with non-unit direction");
    static int objStack[MAX_DEPTH + 1] = {ID_AIR};
    const intersection_t isect = getIntersection(ray);
    int objId;
    if ((objId = isect.objId) < 0) return Eigen::Vector4d::Zero();

    auto &objects = scene.getObjects();
    auto &lights = scene.getLights();
    Eigen::Vector4d I = Eigen::Vector4d::Zero();

    Eigen::Vector4d pointOfIntersection = ray.origin + isect.dist * ray.dir;

    Eigen::Vector4d N = objects[objId]->getUnitNormal(pointOfIntersection);
    Eigen::Vector4d V = -1 * ray.dir;

    auto mat = scene.getMaterial(objects[objId]->matId);

    for (unsigned int i = 0 ; i < lights.size(); ++i) {
        if (lights[i]->isAmbient()) {
            I += mat.Ka * lights[i]->getAmountOfLight(pointOfIntersection).cwiseProduct(mat.rgb);
            continue;
        }
        Eigen::Vector4d L = lights[i]->getVectorToLight(pointOfIntersection);
        Ray rayToLight(pointOfIntersection, L, objId);

        bool lightVisible = true;
        if (lights[i]->getShadowOn()) {
            double distToBlockingObject = getIntersection(rayToLight).dist;
            double distToLight = (pointOfIntersection - lights[i]->getPosition()).norm();

            lightVisible = distToBlockingObject <= EPSILON ||
                distToBlockingObject >= distToLight;
        }
        if (lightVisible) {
            /* Diffuse Reflection */
            Eigen::Vector4d Il = lights[i]->getAmountOfLight(pointOfIntersection);
            // Amount of light visible on surface determined by angle to light source
            double lambertCos = L.dot(N);
            if (lambertCos > 0) {
                I += mat.Kd * lambertCos * mat.rgb.cwiseProduct(Il);
            } else {
                continue;
            }

            /* Specular Reflection */
            Eigen::Vector4d reflection = 2 * N.dot(rayToLight.dir) * N - rayToLight.dir;
            double specCoeff = reflection.dot(V);
            if (specCoeff > 0) {
                specCoeff = std::max(0.0, pow(specCoeff, mat.ns));
                I += specCoeff * mat.Ks * mat.rgb.cwiseProduct(Il);
            }
        }
    }

    if (recursionLevel < MAX_DEPTH) {
        // Work out where in material stack we are
        int nextTransDepth;
        int nextObjId;
        if (objStack[transDepth] == objId) {
            nextTransDepth = transDepth - 1;
            nextObjId = objStack[transDepth - 1];
        } else {
            nextTransDepth = transDepth + 1;
            objStack[nextTransDepth] = objId;
            nextObjId = objId;
        }

        // Compute intensity of reflected ray, if necessary
        if (reflect && mat.Kr > 0)	{
            Ray reflectionRay(pointOfIntersection,  ray.dir - (2 * N.dot(ray.dir)) * N, nextObjId);
            I += mat.Kr * getColor(reflectionRay, recursionLevel + 1, nextTransDepth);
        }

        // Compute intensity of transmission ray, if necessary
        if (refract && mat.Kt > 0) {
            const double etaIncident = (objStack[transDepth] == ID_AIR) ? 1.0 :
                scene.getMaterial(objects[objStack[transDepth]]->matId).Irefract;
            double cosThetaI, etaRefract;
            if (objStack[transDepth] == objId) { // Exiting a material
                cosThetaI = ray.dir.dot(N);
                etaRefract = (objStack[transDepth - 1] == ID_AIR) ? 1.0 :
                    scene.getMaterial(objects[objStack[transDepth - 1]]->matId).Irefract;
            } else {
                cosThetaI = V.dot(N);
                etaRefract = scene.getMaterial(objects[objId]->matId).Irefract;
            }
            const double n = etaIncident/etaRefract;
            const double cosThetaR = sqrt(1 - (n * n) * (1 - cosThetaI * cosThetaI));
            Ray T(pointOfIntersection, (n * ray.dir - (cosThetaR - n * cosThetaI) * N).normalized(), nextObjId);
            I += mat.Kt * getColor(T, recursionLevel + 1, nextTransDepth);
        }
    }

    return I;

}

std::unique_ptr<unsigned char[]> getRaster() {
    const unsigned int row_bytes = scene.cam.viewportWidth * 3;
    const unsigned int buf_len = scene.cam.viewportHeight * row_bytes;
    auto buf = std::unique_ptr<unsigned char[]> (new unsigned char[buf_len]);
    std::vector<Ray, Eigen::aligned_allocator<Ray> > rays(NUM_AA_SUBPIXELS,
                  Ray(Eigen::Vector4d(0, 0, 0, 1), Eigen::Vector4d(1, 0, 0, 0), ID_AIR));
    double multFactor = 1/(1.0 * NUM_AA_SUBPIXELS);
    const Eigen::Vector4d maxrgb(255, 255, 255, 0);
    for (int row = 0; row < scene.cam.viewportHeight; ++row) {
        for (int col = 0; col < scene.cam.viewportWidth; ++col) {
            Eigen::Vector4d color = Eigen::Vector4d::Zero();
            if (!aa) {
                Ray ray = scene.cam.constructRayThroughPixel(col, row);
                color = getColor(ray, 0, 0);
            } else {
                scene.cam.constructRaysThroughPixel(col, row, rays);
                for (int i = 0; i < NUM_AA_SUBPIXELS; ++i) {
                    color += getColor(rays[i], 0, 0);
                }
                color *= multFactor;
            }

            color = 255 * color;
            color = color.cwiseMin(maxrgb);
            buf[row*row_bytes + (col*3)] = color[0];
            buf[row*row_bytes + (col*3) + 1] = color[1];
            buf[row*row_bytes + (col*3) + 2] = color[2];
        }
    }
    return buf;
}

void writePPM(std::unique_ptr<unsigned char[]> buf, unsigned int rows,
              unsigned int cols, std::string filename) {
  std::ofstream ofs (filename, std::ios::out | std::ios::binary);
  ofs << "P6" << "\n";
  ofs << cols << " " << rows << "\n";
  ofs << "255" << "\n";
  ofs.write(reinterpret_cast<const char *> (buf.get()),
            rows*cols*3*sizeof(unsigned char));
  ofs.close();
}
