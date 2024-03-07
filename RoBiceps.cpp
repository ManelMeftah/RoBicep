#include "RoBiceps.h"

RoBiceps::RoBiceps(std::vector<Bone> b) : bones(b), nbBones(size(b)) {}

int RoBiceps::getNbBones()  {
    return nbBones;
}

glm::vec3 RoBiceps::getTarget()  {
    return target;
}

vector<Bone> RoBiceps::getBones() {
    return this->bones;
}

Bone RoBiceps::getBone(int i) {
    return this->bones[i];
}

void RoBiceps::updateAngles(const vector<double>& newAngles) {
    for (size_t i = 0; i < bones.size(); ++i) {
        bones[i].updateAngle(newAngles[i]);
    }
}

arma::mat RoBiceps::calculeJacobienne(const arma::vec& e) {
    arma::vec3 axe_z = arma::vec3{0, 0, 1};

    arma::mat res(3, nbBones); // Initialisation de la matrice résultante

    for (int i = 0; i < nbBones; i++) {
        glm::vec3 bonePos = bones[i].getPosition();
        arma::vec3 armaBonePos = arma::vec3{bonePos.x, bonePos.y, bonePos.z}; // Convertit glm::vec3 en arma::vec3
        arma::vec3 tmp = e - armaBonePos;
        arma::vec3 v = arma::cross(axe_z, tmp); // Calcul du produit vectoriel avec Armadillo
        res.col(i) = arma::conv_to<arma::vec>::from(v); // Convertit arma::vec3 en arma::vec et assigne la colonne
    }


    return res;
}


void RoBiceps::updateBonePositions(std::vector<Bone>& bones) {
    glm::mat4 localRot = glm::mat4(1.0);
    glm::mat4 localTrans = glm::mat4(1.0);

    for (const Bone& b : bones) {
        // Mettre à jour la matrice de rotation locale
        localRot *= b.getRotation();

        // Calculer la nouvelle position du segment
        glm::vec4 position(b.getLongueur(), 0, 0, 1);
        glm::vec4 res = localRot * position;
        localTrans *= glm::translate(glm::mat4(1.0), glm::vec3(res));
    }

    // Mettre à jour les positions de chaque segment
    for (Bone& b : bones) {
        b.setPosition(glm::vec3(localTrans * glm::vec4(0.0, 0.0, 0.0, 1.0)));
    }
}

void RoBiceps::addBone(Bone o) {
    bones.push_back(o);
    nbBones++;
}

double RoBiceps::getLongueurTotale() {
    double lgTot = 0;
    for (int i = 0; i < nbBones; i++)
        lgTot += bones[i].getLongueur();
    return lgTot;
}


// putLinkParameters
void RoBiceps::setArmPosition(arma::vec V) {
    for (int i = 1; i < nbBones; i++) {
        auto newAngle = bones[i].getAngle() + V(i);
        bones[i].updateAngle(newAngle);
    }
}

arma::vec3 RoBiceps::computeEndPoint()
{
    glm::mat4 rotationslocal=mat4(1.0);
    glm::mat4 translationlocal=mat4(1.0);
    glm::vec4 P(0.,0.,0.,1.);
    this->getBone(0).setPosition(glm::vec3{0, 0, 0});

    for(int i=0; i<nbBones; i++)
    {

        Bone b = getBone(i);
        if(i>0)
        {
            glm::vec4 position(getBone(i-1).getLongueur(),0,0,1);
            glm::vec4 res = rotationslocal * position;
            translationlocal *= glm::translate(mat4(1.0), glm::vec3(res));
            getBone(i).setPosition(glm::vec3( translationlocal*P));
        }

        rotationslocal*=b.getRotation();

    }
    glm::vec4 position(getBone(nbBones-1).getLongueur(),0,0,1);
    glm::vec4 res= rotationslocal*position;
    translationlocal *= glm::translate(glm::mat4(1.0), glm::vec3(res));

    glm::vec3 end_point = translationlocal * P;
    arma::vec3 arma_end_point{end_point.x, end_point.y, end_point.z};
    return arma_end_point;
}

// arma::vec3 RoBiceps::computeEndPoint() {
//     glm::mat4 localRot = glm::mat4(1.0);
//     glm::mat4 localTrans = glm::mat4(1.0);

//     // Mettre à jour les transformations locales
//     for (const Bone& b : bones) {
//         // Mettre à jour la matrice de rotation locale
//         localRot *= b.getRotation();

//         // Calculer la nouvelle position du segment
//         glm::vec4 position(b.getLongueur(), 0, 0, 1);
//         glm::vec4 res = localRot * position;
//         localTrans *= glm::translate(glm::mat4(1.0), glm::vec3(res));
//     }

//     // Appliquer les transformations finales
//     glm::vec4 end_point = localTrans * localRot * glm::vec4(0.0, 0.0, 0.0, 1.0);
//     return arma::vec3{end_point.x, end_point.y, end_point.z};
// }

void RoBiceps::display()
{
    mat4 R = mat4(1.0);
    mat4 T = mat4(1.0);
    // cout << getNbBones();
    for(int i=0;i<getNbBones();i++)
    {
      Bone b = getBone(i);
      glPushMatrix();
        if(i>0)
        {
            glm::vec4 position(getBone(i-1).getLongueur(),0,0,1);
            glm::vec4 res = R * position;
            T = T * glm::translate(glm::mat4(1.0), glm::vec3(res));
            glMultMatrixf(&T[0][0]);
        }
        R = R * b.getRotation();
        glMultMatrixf(&R[0][0]);
        b.display();
      glPopMatrix();
    }
}
