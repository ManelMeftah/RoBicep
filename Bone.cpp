#include "Bone.h"
#include <iostream>
#include <GL/glut.h> // ou #include <GLUT/glut.h> si nécessaire

using namespace std;
using namespace glm;

Bone::Bone(double l, double a, const vec3& ax, const mat4& rot, const mat4& trans, const vec3& coul, const vec3& pos)
    : longueur(l), angle(a), axe(ax), rotation(rot), translation(trans), couleur(coul), position(pos) {}

void Bone::update(double lg, double theta)
{
    this->longueur = lg; 
    this->updateAngle(theta);
    this->translation = glm::translate(mat4(1.0), vec3(lg / 2, 0.0f, 0.0f)); 
}

void Bone::updateAngle(const double newAngle) {
    this->angle = newAngle; 
    quat q0;
    q0 = angleAxis((float)radians(this->angle), vec3(0., 0., 1.));
    this->rotation = mat4_cast(q0);
}

void Bone::display() const
{
    glPushMatrix();
    glColor3f(this->couleur.x, this->couleur.y, this->couleur.z); 
    glMultMatrixf(&this->translation[0][0]); 
    glScalef(this->longueur, .2, .2);
    glutSolidCube(1.);
    glPopMatrix();
}

