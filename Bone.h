#pragma once

#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/quaternion.hpp>

#ifdef __APPLE__
#include <GLUT/glut.h> /* Pour Mac OS X */
#else
#include <GL/glut.h>   /* Pour les autres systèmes */
#endif 

using namespace glm;

class Bone {
private:
    double longueur; //longueur de l'os
    double angle; //l'angle de l'os
    vec3 axe; //l'axe de rotation
    mat4 rotation; //l'angle de rotation
    mat4 translation; 
    vec3 couleur; //couleur de l'os
    vec3 position; //position de l'os

public:
    Bone(double l, double a, const vec3& ax, const mat4& rot, const mat4& trans, const vec3& coul, const vec3& pos);
    void update(double lg, double theta);
    void updateAngle(const double newAngle);
    void display() const;

    double getLongueur() const {
        return longueur;
    }

    // Setter pour longueur
    void setLongueur(double l) {
        longueur = l;
    }

    // Getter pour angle
    double getAngle() const {
        return angle;
    }

    // Setter pour angle
    void setAngle(double a) {
        angle = a;
    }

    // Getter pour axe
    vec3 getAxe() const {
        return axe;
    }

    // Setter pour axe
    void setAxe(const vec3& ax) {
        axe = ax;
    }

    // Getter pour rotation
    mat4 getRotation() const {
        return rotation;
    }

    // Setter pour rotation
    void setRotation(const mat4& rot) {
        rotation = rot;
    }

    // Getter pour translation
    mat4 getTranslation() const {
        return translation;
    }

    // Setter pour translation
    void setTranslation(const mat4& trans) {
        translation = trans;
    }

    // Getter pour couleur
    vec3 getCouleur() const {
        return couleur;
    }

    // Setter pour couleur
    void setCouleur(const vec3& coul) {
        couleur = coul;
    }

    // Getter pour position
    vec3 getPosition() const {
        return position;
    }

    // Setter pour position
    void setPosition(const vec3& pos) {
        position = pos;
    }

};
