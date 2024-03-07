#ifndef ROBICEPS_H
#define ROBICEPS_H

#include <iostream>
#include <vector>
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/quaternion.hpp>
#include <glm/gtx/io.hpp>
#include <armadillo>


#include "Bone.h"


using namespace std;
using namespace glm;
using namespace arma;

class RoBiceps {
private:
    vector<Bone> bones; // os qui composent le roBiceps
    int nbBones = 0;
    glm::vec3 target; //cible 

    glm::mat4 R, 
              T;

public:
    RoBiceps(vector<Bone> b);

    void updateAngles(const vector<double>& newAngles);

    int getNbBones() ;
    glm::vec3 getTarget() ;
        //calcule la jacobienne du bras en fonction de l'erreur e
    arma::mat calculeJacobienne(const arma::vec& e);


    vector<Bone> getBones();

    Bone getBone(int i);
    // Mettre à jour les transformations locales
    void updateLocalTransformations(mat4& localRot, mat4& localTrans, const std::vector<Bone>& bones);

    // Calculer la position de la cible
    arma::vec3 computeEndPoint();

    // Mettre à jour les positions de chaque segment du bras
    void updateBonePositions(std::vector<Bone>& bones);

    void addBone(Bone o);

    double getLongueurTotale();

    void setArmPosition(arma::vec V);

    void display();
};

#endif // ROBICEPS_H
