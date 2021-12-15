#include <iostream>
#include <fstream>
#include <vector>
#include <sstream>
#include <math.h>
#include <sys/time.h>
#include <limits>

// GLM includes
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/type_ptr.hpp>
#include <glm/gtx/transform.hpp>
#include <glm/gtx/string_cast.hpp>
#include "glm/detail/_swizzle.hpp"
#include "glm/detail/_swizzle_func.hpp"

// set a constant to test against for refraction
const double airRefractionIDX = 1.000296;

//Section: helper functions
//printGLM function: prints a glm datatype without significant fuss
template <class T>
void printGLM(T val){
    printf("%s\n", glm::to_string(val).c_str());
}
//max function: replaces std::max, though I didn't find out std::max existed until later
// returns the largest of two values
template <class T>
T max(T a, T b){
    if(a > b){
        return a;
    }
    return b;
}
//min function: replaces std::min, though I didn't find out std::min existed until later
// returns the smallest of two values
template <class T>
T min(T a, T b){
    if(a < b){
        return a;
    }
    return b;
}
//cap function: caps all x, y, z values of a dvec3 on a per-axis basis to a specified value
glm::dvec3 cap(glm::dvec3 vec, double val){
    return glm::dvec3(
        min(vec.x, val),
        min(vec.y, val),
        min(vec.z, val)
    );
}
//floor function: on a per-axis basis, ensures a dvec3 cannot be below a specified value
glm::dvec3 floor(glm::dvec3 vec, double val){
    return glm::dvec3(
        max(vec.x, val),
        max(vec.y, val),
        max(vec.z, val)
    );
}
//scale function: finds the largest element of a dvec3, and linear-interpolates all elements so that element == max
// note: not currently in use
glm::dvec3 scale(glm::dvec3 vec, double max){
    double highest = vec.x;
    double scale;
    if(vec.y > highest){
        highest = vec.y;
    }
    if(vec.z > highest){
        highest = vec.z;
    }
    scale = 1.0/highest;
    return vec * scale;
}
// from: https://www.binarytides.com/get-time-difference-in-microtime-in-c/
// allows for more accurate timing information
double time_diff(struct timeval x, struct timeval y){
    double x_ms , y_ms , diff;
	x_ms = (double)x.tv_sec*1000000 + (double)x.tv_usec;
	y_ms = (double)y.tv_sec*1000000 + (double)y.tv_usec;
	diff = (double)y_ms - (double)x_ms;
	return diff;
}
// returns a color based on Phong's BRDF
glm::dvec3 phongBRDF(glm::dvec3 L, glm::dvec3 V, glm::dvec3 N, glm::dvec3 D, glm::dvec3 S, double P){
    double diffuseStrength = glm::dot(N, L);
    glm::dvec3 luminance = D * diffuseStrength;

    glm::dvec3 R = glm::reflect(L, N);
    double specDot = max(glm::dot(R, V), 0.0);

    luminance += S * (double)(std::pow((float)specDot, (float)P));

    return luminance;
}

// Ray struct
// stores all necessary data to perform ray-tracing
struct Ray{
public:
    glm::dvec3 u, v;
    glm::dvec3 u_obj, v_obj;
    glm::dvec3 hit_objspace, hit_worldspace;
    glm::dvec3 normal_w;
    double smallestT, currentRefractionIDX;
    int sphereIDX, recursionDepth;

    // default constructor: a ray ready to be given values and be traced
    Ray(){
        u = glm::dvec3(0.0, 0.0, 0.0);
        v = glm::dvec3(0.0, 0.0, 0.0);
        smallestT = 10000000000.0;
        sphereIDX = -1;
        recursionDepth = 0;
        currentRefractionIDX = 1.0002926;
    }
    // specific constructor: a ray is given values, and can then be traced
    Ray(glm::dvec3 start, glm::dvec3 direction){
        u = start;
        v = direction;
        smallestT = 10000000000.0;
        sphereIDX = -1;
        recursionDepth = 0;
        currentRefractionIDX = 1.0002926;
    }
    // transforms the current vectors by a transformation, and returns a transformed ray
    Ray transform(glm::mat4& transformation){
        return Ray(glm::dvec3(transformation * glm::vec4(u, 1.0)), glm::dvec3(transformation * glm::vec4(v, 0.0)));
    }
    // normalizes the current ray's direction
    void normalize(){
        v = glm::normalize(v);
    }
    // calculates the hitpoints in object and worldspace for the current ray's closest hit
    void calculateHit(glm::mat4 *o2wMatrix){
        hit_objspace = u_obj + v_obj * (double)smallestT;
        hit_worldspace = glm::dvec3(*o2wMatrix * glm::vec4(hit_objspace, 1.0));
        return;
    }
    // calculates the normal in worldspace from the ray's closest hitpoint in objectspace
    void calculateNormal_w(glm::mat4 *w2oMatrix){
        normal_w = glm::dvec3(glm::transpose(*w2oMatrix) * glm::vec4(hit_objspace, 0.0));
        normal_w = glm::normalize(normal_w);
    }
};

// material struct to be given to spheres
// holds Kd, Ks, Kr, phong coefficient, and Kt
struct Material{
public:
    glm::dvec3 diffuse;
    glm::dvec3 specular;
    glm::dvec3 refraction;
    double refractiveIndex;
    double phong;
    bool refractive;

    Material(){
        diffuse = glm::dvec3(0.0, 0.0, 0.0);
        specular = glm::dvec3(0.0, 0.0, 0.0);
        phong = 0;
        refraction = glm::dvec3(0.0, 0.0, 0.0);
        refractiveIndex = 1.0002926;
        refractive = false;
    }
};

// rendererState class
// holds information and methods necessary to read data from a scn file,
// such that they can be easily interpreted by the program
class rendererState{
public:
    // these std::vectors act as stacks for group/groupend
    std::vector<glm::mat4> o2wMatrices;
    std::vector<glm::mat4> w2oMatrices;
    std::vector<Material> materials;
    glm::dvec3 ambient;
    glm::dvec3 backgroundColor;
    int imageSize, num_samples;
    double planeDist;
    bool superSampling;

    // default constructor starts with default values
    rendererState(){
        o2wMatrices.push_back(glm::mat4(1.0));
        w2oMatrices.push_back(glm::mat4(1.0));
        materials.push_back(Material());
        backgroundColor = glm::dvec3(0.0, 0.0, 0.0);
        superSampling = false;
        num_samples = 0;
    }
    // sets supersampling values
    void setSuperSampling(int samples){
        superSampling = true;
        num_samples = samples;
    }
    // sets imageplane information
    void setView(int n, double d){
        imageSize = n;
        planeDist = d;
    }
    // adds a transformation to the current top of the stacks
    void addTrans(glm::mat4 o2wTrans, glm::mat4 w2oTrans){
        o2wMatrices.back() = o2wMatrices.back() * o2wTrans;
        w2oMatrices.back() = w2oTrans * w2oMatrices.back();
    }
    // sets background color
    void setBackground(glm::dvec3 color){
        backgroundColor = color;
    }
    // sets ambient color
    void setAmbient(glm::dvec3 color){
        ambient = color;
    }
    // sets a new material, not that refraction is not necessary here
    void setMaterial(glm::dvec3 d, glm::dvec3 s, double p){
        materials.back().diffuse = d;
        materials.back().specular = s;
        materials.back().phong = p;
    }
    // sets refraction information in a material
    void setRefraction(glm::dvec3 r, double i){
        materials.back().refraction = r;
        materials.back().refractiveIndex = i;
        materials.back().refractive = true;
    }
    // pushes a copy of the current matrices onto the top of the stacks
    void group(){
        o2wMatrices.push_back(o2wMatrices.back());
        w2oMatrices.push_back(w2oMatrices.back());
    }
    // pops the top of the stacks
    void groupEnd(){
        o2wMatrices.pop_back();
        w2oMatrices.pop_back();
    }
};

// Sphere class
// holds object-to-world and world-to-object matrices for intersection calculations
// also does most of the intersection work, and keeps a material
class Sphere{
public:
    glm::mat4 o2wMatrix, w2oMatrix;
    Material material;

    Sphere(glm::mat4 o2w, glm::mat4 w2o, Material mat){
        o2wMatrix = o2w;
        w2oMatrix = w2o;
        material = mat;
    }
    // checks if a ray intersects with this sphere
    // returns true if yes, and sets the ray's closest intersection value if necessary
    bool intersect(Ray &ray, int idx){
        // transform the ray into object space
        Ray tempray = ray.transform(w2oMatrix);

        // essentially just the quadratic formula
        double a = glm::dot(tempray.v, tempray.v);
        double b = 2*glm::dot(tempray.u, tempray.v);
        double c = glm::dot(tempray.u, tempray.u) - 1.0;

        double D = b*b - 4.0*a*c;
        if(D <= 0.0){
            return false;
        }
        double rootD = std::sqrt(D);
        double t0 = 0.5 * (-b - rootD)/a;
        double t1 = 0.5 * (-b + rootD)/a;
        //note that t0 will always be smaller than t1
        if(t0 > 0.0 && t0 <= ray.smallestT){
            // set relevant information in the ray
            ray.smallestT = t0;
            ray.sphereIDX = idx;
            ray.u_obj = tempray.u;
            ray.v_obj = tempray.v;
            return true;
        }
        if(t1 > 0.0 && t1 <= ray.smallestT){
            // set relevant information in the ray
            ray.smallestT = t1;
            ray.sphereIDX = idx;
            ray.u_obj = tempray.u;
            ray.v_obj = tempray.v;
            return true;
        }
        return false;
    }
};

// Light struct
// holds all information necessary for a light source with hard shadows
struct Light{
public:
    glm::dvec3 color, pos_w;
    //o2w and w2o matrices probably not necessary
    glm::mat4 o2wMatrix;
    glm::mat4 w2oMatrix;
    //constructor requires a color and a position in worldspace
    Light(glm::dvec3 col, glm::dvec3 position){
        o2wMatrix = glm::translate(position);
        w2oMatrix = glm::translate(-position);
        pos_w = position;
        color = col;
    }
    // checkIntersections method
    // looks at a vector of spheres and a hitpoint, and checks if any spheres are in the way
    bool checkIntersections(glm::dvec3 hit_w, std::vector<Sphere> *spheres){
        Ray lightRay = Ray(pos_w, hit_w - pos_w);
        // in the future i would like to optimize this
        for(auto sphere: *spheres){
            sphere.intersect(lightRay, 0.0);
            //shortcut for-loop if there's anything in the way
            if(lightRay.smallestT < 0.99999){
                return true;
            }
        }
        return false;
    }
};

// calculate illumination function
// uses knowledge of all spheres and lights to determine each light's contribution
// to the color at a specific point
// uses Light.checkIntersections and phongBRDF()
glm::dvec3 calculateIllumination(glm::dvec3 *ambient, glm::dvec3 *diffuse, glm::dvec3 *specular, double phong, std::vector<Light> *lights, glm::dvec3 hit_w, glm::dvec3 normal_w, std::vector<Sphere> *spheres, glm::dvec3 *viewDir){
    glm::dvec3 luminance = *ambient;
    for(auto light : *lights){
        //static double inverseNumLights = 1.0f / (double)lights->size();
        bool blocked = light.checkIntersections(hit_w, spheres);
        if(!blocked){
            glm::dvec3 lightDir = glm::normalize(light.pos_w - hit_w );
            double illuminance = glm::dot(lightDir, normal_w);
            if(illuminance > 0.0){
                luminance += /*inverseNumLights **/ light.color * phongBRDF(
                    lightDir,
                    *viewDir,
                    normal_w,
                    *diffuse,
                    *specular,
                    phong
                );
            }
        }
    }
    return luminance;
}

// I don't want these to be global, but they're okay as they are
// rendererState and list of spheres & lights for file I/O
rendererState state = rendererState();
std::vector<Sphere> spheres;
std::vector<Light> lights;

// readInput function
void readInput(char * fileName){
    //open file
    std::ifstream infile(fileName);
    std::string line;
    std::string tok;
    
    //temporary variables to read into
    double a, b, c, d, e, f, g;
    //read each line
    while(std::getline(infile, line)){
        std::istringstream tokens(line);
        //separate each line by its whitespace
        while(tokens >> tok){
            // break immediately and get a new line if comment
            if(tok[0] == '#'){
                break;
            }
            // set supersampling information
            else if(tok == "supersample"){
                tokens >> a;
                state.setSuperSampling(a);
            }
            // set view properties
            if(tok == "view"){
                tokens >> a >> b;
                state.setView(a, b);
            }
            // add a new scale to rendererstate
            else if(tok == "scale"){
                tokens >> a >> b >> c;
                state.addTrans(
                    glm::scale(glm::dvec3(a, b, c)),
                    glm::scale(glm::dvec3(1.0/a, 1.0/b, 1.0/c))
                );
            }
            // add a new translate to rendererstate
            else if(tok == "move"){
                tokens >> a >> b >> c;
                state.addTrans(
                    glm::translate(glm::dvec3(a, b, c)),
                    glm::translate(glm::dvec3(-1.0*a, -1.0*b, -1.0*c))
                );
            }
            // add a new rotate to rendererstate
            else if(tok == "rotate"){
                tokens >> a >> b >> c >> d;
                state.addTrans(
                    glm::rotate(glm::radians((double)a), glm::dvec3(b, c, d)),
                    glm::rotate(-1.0f*glm::radians((double)a), glm::dvec3(b, c, d))
                );
            }
            // set background information
            else if(tok == "background"){
                tokens >> a >> b >> c;
                state.setBackground(glm::dvec3(a, b, c));
            }
            // set ambient information
            else if(tok == "ambient"){
                tokens >> a >> b >> c;
                state.setAmbient(glm::dvec3(a, b, c));
            }
            // set material information
            else if(tok == "material"){
                tokens >> a >> b >> c >> d >> e >> f >> g;
                state.setMaterial(glm::dvec3(a, b, c), glm::dvec3(d, e, f), g);
            }
            // set refraction information
            else if(tok == "refraction"){
                tokens >> a >> b >> c >> d;
                state.setRefraction(glm::dvec3(a, b, c), d);
            }
            // tell state to push a new group onto the stack
            else if(tok == "group"){
                state.group();
            }
            // tell state to pop a group off the stack
            else if(tok == "groupend"){
                state.groupEnd();
            }
            // create a new sphere
            else if(tok == "sphere"){
                spheres.push_back(
                    Sphere(
                        state.o2wMatrices.back(),
                        state.w2oMatrices.back(), 
                        state.materials.back())
                    );
            }
            // create a new light
            else if(tok == "light"){
                tokens >> a >> b >> c >> d >> e >> f;
                lights.push_back(Light(glm::dvec3(a, b, c), glm::dvec3(d, e, f)));
            }
        }
    }
}

// function header for recursion
// all functions should have headers, but I'm low on time
glm::dvec3 trace(Ray);

// refract function
// takes a ray, and refracts it based on the sphere it hit
Ray refract(const Ray &r) {
    double eta, cosi, k;
    Ray outRay = Ray();

    // This is a *massive* oversimplification, but assume no spheres intersect
    // if the refraction index is already that of air, assume we're going into a sphere
    if(r.currentRefractionIDX == airRefractionIDX){
        eta = airRefractionIDX/spheres[r.sphereIDX].material.refractiveIndex;
        outRay.currentRefractionIDX = spheres[r.sphereIDX].material.refractiveIndex;
    }
    // if the refraction index isn't that of air, assume we're leaving a sphere (and going into air)
    else{
        eta = r.currentRefractionIDX/airRefractionIDX;
        outRay.currentRefractionIDX = airRefractionIDX;
    }

    // calculation obtained from https://github.com/ssloy/tinyraytracer/blob/master/tinyraytracer.cpp
    // for some reason the one given in the lectures returned extremely large directions
    glm::dvec3 I = -r.v;
    cosi = -std::max(-1.0, std::min(1.0, glm::dot(I, r.normal_w)));
    k = 1 - eta*eta*(1.0 - cosi*cosi);

    outRay.v = glm::dvec3(I*eta + r.normal_w*(eta*cosi - std::sqrt(k)));
    outRay.u = r.u + outRay.v*0.0001;
    outRay.recursionDepth = r.recursionDepth+1;
    // return a refracted ray at a deeper level of recursion
    return outRay;
}

//reflect function
// takes a ray and reflects it off of its hitpoint
// returns a reflected ray at a deeper level of recursion
Ray reflect(const Ray &r){
    Ray reflectedRay = Ray(r.hit_worldspace + r.normal_w*0.00001, glm::reflect(r.v, r.normal_w));
    reflectedRay.recursionDepth = r.recursionDepth + 1;
    return reflectedRay;
}

// Shade function
// calculates illumination at a hitpoint, and recurses based on reflection and refraction
glm::dvec3 Shade(Ray &ray){
    Sphere *currentSphere = &spheres[ray.sphereIDX];
    // lazily calculate hit information now that it's necessary
    ray.calculateHit(&spheres[ray.sphereIDX].o2wMatrix);
    ray.calculateNormal_w(&spheres[ray.sphereIDX].w2oMatrix);

    // get a color 
    glm::dvec3 color = calculateIllumination(
        &state.ambient,
        &currentSphere->material.diffuse,
        &currentSphere->material.specular,
        currentSphere->material.phong,
        &lights,
        ray.hit_worldspace,
        ray.normal_w,
        &spheres,
        &ray.v
    );

    // only send 6 reflections/refractions
    if(ray.recursionDepth < 6){
        color += currentSphere->material.specular * trace(reflect(ray));
        
        //only send a refraction ray if the material is refractive
        if(currentSphere->material.refractive){
            color += currentSphere->material.refraction * trace(refract(ray));
        }
    }
    // return the color at the intersection
    return color;
}

// Trace function
// checks intersection of a ray and all spheres, and returns the color at any hitpoints
// if no hitpoints, returns the background color
glm::dvec3 trace(Ray ray){
    const unsigned int numSpheres = spheres.size();

    for(unsigned int k = 0; k < numSpheres; k++){
        spheres[k].intersect(ray, k);
    }
    if(ray.sphereIDX >= 0){
        glm::dvec3 luminance = Shade(ray);
        return luminance;
    }
    return state.backgroundColor;
}

// superSample function
// adds another pair of for-loops to send more rays at varying places in a pixel for more accurate sub-pixel data
glm::ivec3 superSample(int num_samples, glm::dvec3 eyeLoc, double pixelSize, glm::dvec3 pixelLocation){
    // intialize a vector to hold samples
    // i would like this to be more static, but omp's parallel for didn't want to work with that
    std::vector<glm::dvec3>sampleArray = std::vector<glm::dvec3>();

    double subSampleSize = pixelSize/state.num_samples;
    for(int k = 0; k < state.num_samples; k++){
        for(int l = 0; l < state.num_samples; l++){
            // jitter pixel location for each sub-pixel ray
            glm::dvec3 subPixelLocation = glm::vec3(
                (pixelLocation.x-subSampleSize*l), 
                (pixelLocation.y+subSampleSize*k), 
                pixelLocation.z
            );
            Ray ray = Ray(eyeLoc, subPixelLocation - eyeLoc);
            ray.normalize();
            sampleArray.push_back(trace(ray));
        }
    }
    // calculate average color of all rays sent
    glm::dvec3 averageColor, totalColor = glm::dvec3(0, 0, 0);
    for(auto sample: sampleArray){
        totalColor += sample;
    }
    averageColor = totalColor/(double)sampleArray.size();

    // return average color of a pixel
    return floor(cap(averageColor * 254.0, 255.0), 0.0);
}

//RayCast function
// generates an output array, and sends rays based on the rendererState's information
// initiates raycasting
std::vector<std::vector<glm::ivec3>> RayCast(){
    //TODO: replace with just a 2d array now that output is sorted out
    std::vector<std::vector<glm::ivec3>> outputArray;
    for(int i = 0; i < state.imageSize; i++){
        outputArray.push_back(std::vector<glm::ivec3>());
        for(int j = state.imageSize; j >= 0; j--){
            outputArray[i].push_back(glm::ivec3(state.backgroundColor*255.0));
            // printf("%s\n", glm::to_string(outputArray[i].back()).c_str());
        }
    }

    //information for progress reports in console
    int counter = 0;
    int counterRate = 20000000 / spheres.size();
    const double totalRays = state.imageSize * state.imageSize;
    const double rayPercentage = 1.0/(double)totalRays;

    // determine pixel size on the image plane
    const double pixelSize = (2.0*state.planeDist)/state.imageSize;
    const double startX = -state.planeDist + (pixelSize*0.5);
    const double startY = state.planeDist - (pixelSize*0.5);
    // eye is always at 0 0 1
    const glm::dvec3 eyeLoc = glm::dvec3(0.0, 0.0, 1.0);
    // image plane is always on the -z side of the eye
    const double startZ = eyeLoc.z - 1.0;

    glm::dvec3 pixelLocation;

    // formatting line
    printf("\n");
    
    // begin parallelism
    #pragma omp parallel for num_threads(256)
    //loop through all horizontal pixels
    for(int i = 0; i < state.imageSize; i++){
        //loop through all vertical pixels
        for(int j = 0; j < state.imageSize; j++){
            // print progress information periodically
            if(counter%counterRate == 1){
                printf("\033[A\33[2K%f percent done\n", 100.0 * (double)counter * rayPercentage);
            }
            // determine location of the specific pixel
            pixelLocation = glm::dvec3(
                startX + (pixelSize*j),
                startY - (pixelSize*i),
                startZ
            );
            
            // jitter pixels at a sub-pixel level if supersampling is active
            if(state.superSampling == true){
                outputArray[j][i] = superSample(state.num_samples, eyeLoc, pixelSize, pixelLocation);
            }
            // otherwise just send a single primary ray per pixel
            else{
                Ray ray = Ray(eyeLoc, pixelLocation - eyeLoc);
                ray.normalize();
                outputArray[j][i] = floor(cap(trace(ray) * 254.0, 255.0), 0.0);
            }
            counter++;
        }
    }
    // return an array containing information about all pixel's colors
    return outputArray;
}

int main(int argc, char * argv[])
{
    // get validity information of arguments
    if(argc < 2){
        std::cout << "Please provide an input file\n";
        return 1;
    }
    else if(argc > 2){
        std::cout << "Too many arguments provided\n";
        return 1;
    }

    // read in a .scn file
    readInput(argv[1]);

    // get timing data to time the image generation
    struct timeval startTime, endTime;
    gettimeofday(&startTime, NULL);

    // cast all necessary rays
    std::vector<std::vector<glm::ivec3>> outputArray = RayCast();

    // finish timing data for image rendering
    gettimeofday(&endTime, NULL);
    printf("\033[A\33[2KRender complete in: %f seconds\n", time_diff(startTime , endTime) / 1000000.0);

    //begin writing to file
    printf("Writing to file...\n");
    //also time this out of curiosity
    gettimeofday(&startTime, NULL);

    FILE *picfile;
	picfile = fopen("out.ppm", "w");
	fprintf(picfile, "P6\n# %dx%d Raytracer output\n%d %d\n255\n",
                state.imageSize, state.imageSize, state.imageSize, state.imageSize);
    // For each pixel, save information to the file
	for (int j=0; j < state.imageSize; j++) {    // Y is flipped!
	    for (int i=0; i < state.imageSize; i++) {
	        fprintf(picfile, "%c%c%c",outputArray[i][j].r,outputArray[i][j].g,outputArray[i][j].b);
	    }
    }
    fclose(picfile);
    // save timing information, and output file writing time
    gettimeofday(&endTime, NULL);
    // file should now be readable
    printf("\033[A\33[2KFinished writing to file in %f\n", time_diff(startTime, endTime) / 1000000.0);

    return 0;
}