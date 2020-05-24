//
// Created by Göksu Güvendiren on 2019-05-14.
//

#include "Scene.hpp"

void Scene::buildBVH() {
    printf(" - Generating BVH...\n\n");
    this->bvh = new BVHAccel(objects, 1, BVHAccel::SplitMethod::NAIVE);
}

Intersection Scene::intersect(const Ray &ray) const
{
    return this->bvh->Intersect(ray);
}

void Scene::sampleLight(Intersection &pos, float &pdf) const
{
    float emit_area_sum = 0;
    for (uint32_t k = 0; k < objects.size(); ++k) {
        if (objects[k]->hasEmit()){
            emit_area_sum += objects[k]->getArea();
        }
    }
    float p = get_random_float() * emit_area_sum;
    emit_area_sum = 0;
    for (uint32_t k = 0; k < objects.size(); ++k) {
        if (objects[k]->hasEmit()){
            emit_area_sum += objects[k]->getArea();
            if (p <= emit_area_sum){
                objects[k]->Sample(pos, pdf);
                break;
            }
        }
    }
}

bool Scene::trace(
        const Ray &ray,
        const std::vector<Object*> &objects,
        float &tNear, uint32_t &index, Object **hitObject)
{
    *hitObject = nullptr;
    for (uint32_t k = 0; k < objects.size(); ++k) {
        float tNearK = kInfinity;
        uint32_t indexK;
        Vector2f uvK;
        if (objects[k]->intersect(ray, tNearK, indexK) && tNearK < tNear) {
            *hitObject = objects[k];
            tNear = tNearK;
            index = indexK;
        }
    }


    return (*hitObject != nullptr);
}

// Implementation of Path Tracing
Vector3f Scene::castRay(const Ray &ray, int depth) const
{
    // TODO Implement Path Tracing Algorithm here
    if (depth > this->maxDepth) {
      return Vector3f(0.0,0.0,0.0);
    }
    
    Intersection intersection = Scene::intersect(ray);
    Vector3f hitColor = this->backgroundColor;
    if(!intersection.happened)
      return hitColor;
    if (intersection.m->hasEmission())
      return Vector3f(1.0,1.0,1.0);
    
    // contribution from the light source
    Vector3f dir_color = Vector3f(0, 0, 0);
    float pdf_light;
    Intersection lightPoint;
    sampleLight(lightPoint, pdf_light);
    lightPoint.normal.normalized();
  
    Vector3f w_dir = normalize(lightPoint.coords - intersection.coords);
    Ray shadowRay(intersection.coords, w_dir);
    Intersection shadowRayInter = Scene::intersect(shadowRay);
    
    // light ray not blocked in the middle
    if (!intersection.m->hasEmission())
    {
      if (!shadowRayInter.happened || shadowRayInter.m->hasEmission())
      {
        if (pdf_light < EPSILON)
          pdf_light = EPSILON;
    
        Vector3f f_r1 = intersection.m->eval(-ray.direction, w_dir, intersection.normal);
        float kk = dotProduct(intersection.coords - lightPoint.coords, intersection.coords - lightPoint.coords);
        dir_color = lightPoint.emit * f_r1 * dotProduct(w_dir, intersection.normal)
                    * dotProduct(-w_dir, lightPoint.normal) / kk / pdf_light;
      }
    }
    
    
    
    // contribution from other objects
    // Russian Roulette
    Vector3f indir_color = Vector3f(0,0,0);
    float testrr = get_random_float();
    if (testrr <= RussianRoulette)
    {
        Vector3f randomDir = intersection.m->sample(-ray.direction, intersection.normal);
        randomDir = randomDir.normalized();
        float pdf_object = intersection.m->pdf(-ray.direction, randomDir,intersection.normal);
        Ray ro(intersection.coords, randomDir);
        
        Intersection objRayInter = Scene::intersect(ro);
        if (objRayInter.happened)
          if (!objRayInter.obj->hasEmit())
          {
            if (pdf_object < EPSILON)
              pdf_object = EPSILON;
            
            Vector3f f_r2 = intersection.m->eval(-ray.direction, ro.direction, intersection.normal);
            indir_color = castRay(ro, ++depth) * f_r2
                    * dotProduct(ro.direction, intersection.normal) / pdf_object /RussianRoulette;
          }
        
    }
    
    return dir_color + indir_color;
}
