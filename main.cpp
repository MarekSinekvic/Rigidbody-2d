#include <SFML/Graphics.hpp>
#include <iostream>
#include <math.h>
#include <thread>
#include <vector>

using namespace sf;
using namespace std;

const float PI = 3.14159265358979323846f;
int WIDTH = 800;
int HEIGHT = 600;

#include "../../Libraries/Library.h"
Camera mainCamera = Camera();
Font defaultFont;

Vector2f getNormal(Vector2f a)
{
    return Vector2f(-a.y, a.x);
}
int frame = 0;
float deltaTime = 0;

class Body {

public:
    Vector2f position, velocity;
    float rotation, rotVelocity;
    float mass, restitution, friction, inertiaMoment;
    float radius;
    Color color;
    Body(Vector2f StartPosition = Vector2f(0,0), Vector2f StartVelocity = Vector2f(0.0f,0.0f), float Rotation = 0.0f, float RotationVelocity = 0.0f, float Mass = 1.0f, float Restitution = 0.5f, float Friction = 0.7f, Color color = Color::White) {
        this->position = StartPosition;
        this->velocity = StartVelocity;
        this->rotation = Rotation;
        this->rotVelocity = RotationVelocity;
        
        this->mass = Mass;
        this->restitution = Restitution;
        this->friction = Friction;

        this->inertiaMoment = 1.0f;

        this->color = color;
    }
    Vector2f getVelocityOfPoint(Vector2f p) {
        Vector2f normal = getNormal(p-this->position);
        return this->velocity + normal * this->rotVelocity;
    }
    void AddForce(Vector2f origin, Vector2f force) {
        Vector2f delta = origin - this->position;
        this->velocity += force / this->mass;
        this->rotVelocity += magnitude(delta)*magnitude(force)*dot(normalize(force), normalize(getNormal(delta)))/this->inertiaMoment;
    }
    void AddCollisionForce(Vector2f collisionPosition, Vector2f forceDirection) {
        Vector2f fullVelocity = this->getVelocityOfPoint(collisionPosition);
        float projectedV = (-dot(fullVelocity,forceDirection));
        if (projectedV < 0) projectedV = 0;

        this->AddForce(collisionPosition, forceDirection*this->mass*(projectedV*(this->restitution+1.0f)));

        Vector2f frictionDirection = getNormal(forceDirection);
        frictionDirection = (dot(fullVelocity, frictionDirection) > 0) ? -frictionDirection : frictionDirection;
        this->AddForce(collisionPosition, frictionDirection*this->mass*abs(dot(normalize(fullVelocity), frictionDirection))*this->friction);
    }
};

class Circle : public Body {
private:
    CircleShape circleRender;
    VertexArray debugLines;

public:
    float radius;
    Circle(float Radius = 1.0f) {
        this->inertiaMoment = this->mass*Radius*Radius/2.0f;

        this->radius = Radius;

        this->circleRender = CircleShape(this->radius);
        this->circleRender.setOrigin(this->radius,this->radius);
        this->circleRender.setOutlineColor(this->color);
        this->circleRender.setFillColor(Color::Transparent);
        this->circleRender.setOutlineThickness(1.0f);

        this->debugLines = VertexArray(Lines);
    }
    void Update() {
        this->position += this->velocity * deltaTime;
        this->rotation += this->rotVelocity * deltaTime;
        this->WorldPhysics();
    }
    void Draw(RenderWindow *window) {
        this->circleRender.setPosition(this->position);
        window->draw(this->circleRender);

        this->debugLines.append(Vertex(this->position,Color::Red));
        this->debugLines.append(Vertex(this->position + Vector2f(cosf(this->rotation),sinf(this->rotation))*this->radius,Color::Red));
        window->draw(this->debugLines);
        this->debugLines.clear();
    }
    void WorldPhysics() {
        this->velocity.y += 50.0f*deltaTime;
        this->WallsCollision();
    }
    void CirclesCollision(Vector2f otherPosition, float penetration) {
        Vector2f normal = normalize(otherPosition - this->position);
        Vector2f pos = this->position + normal*(this->radius);
        Vector2f forceDirection = -normal;

        Vector2f fullVelocity = this->getVelocityOfPoint(pos);
        float projectedV = (-dot(fullVelocity,forceDirection));
        if (projectedV < 0) projectedV = 0;

        this->AddForce(pos, forceDirection*this->mass*(projectedV*(this->restitution+1.0f)));

        Vector2f frictionDirection = getNormal(forceDirection);
        frictionDirection = (dot(fullVelocity, frictionDirection) > 0) ? -frictionDirection : frictionDirection;
        this->AddForce(pos, frictionDirection*this->mass*abs(dot(normalize(fullVelocity), frictionDirection))*this->friction);

        this->position += -normal * (penetration);

        this->debugLines.append(Vertex(pos,Color::Green));
        this->debugLines.append(Vertex(pos + forceDirection*projectedV,Color::Green));
    }
    void WallsCollision() {
        Vector2f forceDirection = Vector2f(0,0);
        
        if (this->position.x-this->radius < 0) {
            forceDirection = Vector2f(1,0);
            this->position.x = this->radius;
        }
        if (this->position.x+this->radius > WIDTH) {
            forceDirection = Vector2f(-1,0);
            this->position.x = WIDTH-this->radius;
        }
        if (this->position.y-this->radius < 0) {
            forceDirection = Vector2f(0,1);
            this->position.y = this->radius;
        }
        if (this->position.y+this->radius > HEIGHT) {
            forceDirection = Vector2f(0,-1);
            this->position.y = HEIGHT-this->radius;
        }
        
        if (magnitude(forceDirection) > 0) {
            Vector2f pos = this->position - forceDirection * this->radius;

            Vector2f fullVelocity = this->getVelocityOfPoint(pos);
            float projectedV = (-dot(fullVelocity,forceDirection));
            if (projectedV < 0) projectedV = 0;

            this->AddForce(pos, forceDirection*this->mass*(projectedV*(this->restitution+1.0f)));

            Vector2f frictionDirection = getNormal(forceDirection);
            frictionDirection = (dot(fullVelocity, frictionDirection) > 0) ? -frictionDirection : frictionDirection;
            this->AddForce(pos, frictionDirection*this->mass*abs(dot(normalize(fullVelocity), frictionDirection))*this->friction);
        }
    }
};

class RigidBody
{
private:
    VertexArray debugPoints;
    VertexArray debugVectors;
    Text debugText;

    float TrapezeInertiaMoment(Vector2f target, float h1, float h2, float w) {
        float deltaH = h2-h1;

        float v1 = powf(w,3.)*((powf(h2,3.)-powf(h1,3.))/3.-h1*h2*h2+h1*h1*h2+deltaH)/4.;
        float v2 = powf(w,2.)*(deltaH*deltaH*(h1-target.y)+h1*(1+2*target.x)-2*h2*target.x)/3.;
        float v3 = powf(w,1.)*(h2*target.x*target.x-target.x*h1*(target.x+2)+deltaH*powf(target.y-h1,2.))/2.;
        float v4 = ((h1*h1*h1)/3.-h1*h1*target.y+h1*(powf(target.x,2.)+powf(target.y,2.)));

        return (v1+v2+v3+v4)/(h1+deltaH*w/2.);
    }
public:
    VertexArray verts;
    Vector2f position, velocity;
    float rotation, rotVelocity;
    float inertiaMoment = 0.0f, mass = 1.0f;

    Vector2f center;
    RigidBody(VertexArray vertexes, Vector2f StartPosition, float StartRotation, float mass = 1.0f, float inertiaMoment = 1.0f)
    {
        this->verts = vertexes;

        this->center = Vector2f(0, 0);
        for (int i = 0; i < this->verts.getVertexCount(); i++)
        {
            this->center += this->verts[i].position;
            float dst = magnitude(this->verts[i].position);
        }
        this->center /= (float)this->verts.getVertexCount();
        
        this->mass = mass;

        this->inertiaMoment = inertiaMoment * this->mass;

        this->position = StartPosition;
        this->rotation = StartRotation; 

        this->velocity = Vector2f(0, 0);
        this->rotVelocity = 0;

        this->debugPoints = VertexArray(Points);
        this->debugPoints.append(Vertex(this->center + this->position, Color::Green));
        this->debugPoints.append(Vertex(this->position, Color::Blue));

        this->debugVectors = VertexArray(Lines);

        this->debugText.setFillColor(Color::Green);
        this->debugText.setFont(defaultFont);
        this->debugText.setCharacterSize(12);
        this->debugText.setPosition(10, 200);

        cout << this->TrapezeInertiaMoment(Vector2f(0.9f,1.5f),3.,4.12f,1.8f) << endl;
    }
    bool isPointInside(Vector3f point) {
        
    }
    float calculateInertiaMoment(Vector2f center) {
        
        return 0.;
    }
    Vector2f GetVertexGlobalPosition(int i)
    {
        Vector2f delta = verts[i].position - this->center;
        float magn = magnitude(delta);

        float cos = (delta.x) / (magn);
        float cang = acosf(cos);
        if (delta.y < 0)
            cang = -cang;

        Vector2f pos = this->center + (Vector2f(cosf(cang + this->rotation), sinf(cang + this->rotation))) * magn + this->position;

        return pos;
    }
    Vector2f getVelocityOfPoint(Vector2f p) {
        Vector2f normal = getNormal(p-this->position);
        return this->velocity + normal * this->rotVelocity;
    }
    void Draw(RenderWindow *window)
    {
        VertexArray renderVerts(this->verts);
        for (int i = 0; i < renderVerts.getVertexCount(); i++)
        {
            renderVerts[i].position = GetVertexGlobalPosition(i);
        }
        renderVerts.append(renderVerts[0]);
        window->draw(renderVerts);

        window->draw(this->debugPoints);

        this->debugText.setPosition(this->position);
        this->debugText.setString(to_string(this->rotVelocity));
        window->draw(this->debugText);

        this->debugVectors.clear();
        for (int i = 0; i < this->verts.getVertexCount(); i++) {
            Vector2f P = GetVertexGlobalPosition(i);
            // this->debugVectors.append(Vertex(P,Color::Blue));
            // this->debugVectors.append(Vertex(P+this->getVelocityOfPoint(P)*10.0f,Color::Blue));
        }
        this->debugVectors.append(Vertex(this->position,Color::Blue));
        this->debugVectors.append(Vertex(this->GetVertexGlobalPosition(0),Color::Blue));
        window->draw(this->debugVectors);
    }
    void AddForceAt(Vector2f origin, Vector2f force)
    {
        Vector2f delta = origin - (this->position + this->center);
        // float startV = this->rotVelocity;
        // float startV2 = magnitude(this->velocity);
        this->velocity += force / this->mass;
        this->rotVelocity += magnitude(delta)*magnitude(force)*dot(normalize(force), normalize(getNormal(delta)))/this->inertiaMoment;
        // cout << "frame: " << frame << endl;
        // cout << "rotation: " << (magnitude(delta)*magnitude(force)*dot(normalize(force), normalize(getNormal(delta)))/this->inertiaMoment) << " | " << startV << " -> " << this->rotVelocity << endl;
        // cout << "linear: " << (magnitude(force) / this->mass) << " | " << startV2 << " -> " << magnitude(this->velocity) << endl;
    }
    void WallsCollide()
    {
        int downTouchCount = 0, upTouchCount = 0;
        int rightTouchCount = 0, leftTouchCount = 0;

        float xPenetration = 0;
        float yPenetration = 0;
        for (int i = 0; i < this->verts.getVertexCount(); i++)
        {
            Vector2f pos = this->GetVertexGlobalPosition(i);
            
            Vector2f forceDirection = Vector2f(0.0f,0.0f);
            float penetration = 0.0f;
            if (pos.x < 0)
            {
                forceDirection = Vector2f(1,0);
                penetration = -pos.x;

                this->position.x += (-pos.x);
                xPenetration += -pos.x;
                leftTouchCount++;
            }
            if (pos.x > WIDTH)
            {
                forceDirection = Vector2f(-1,0);
                penetration = pos.x-WIDTH;

                this->position.x += (WIDTH - pos.x);
                xPenetration += WIDTH - pos.x;
                if (xPenetration)
                    rightTouchCount++;
            }
            if (pos.y < 0 && false)
            {
                this->AddForceAt(pos, Vector2f(0, this->velocity.y) * 2.0f * 0.7f);
                this->AddForceAt(pos, Vector2f(dot(Vector2f(1, 0), this->velocity) * 0.04, 0));

                this->position.y += (-pos.y);
                xPenetration = -pos.x;
                upTouchCount++;
            }
            if (pos.y > HEIGHT)
            {
                forceDirection = Vector2f(0,-1);
                penetration = pos.y-HEIGHT;

                // cout << this->velocity.x << endl;

                this->position.y += (HEIGHT - pos.y);
                xPenetration = -pos.x;
                leftTouchCount++;
            }

            Vector2f fullVelocity = this->getVelocityOfPoint(pos);

            float projectedV = (-dot(fullVelocity,forceDirection));
            if (projectedV < 0) projectedV = 0;
            if (magnitude(forceDirection) > 0) {
                this->AddForceAt(pos, forceDirection*this->mass*(projectedV*0.7f));
                // cout << projectedV << endl;

                Vector2f frictionDirection = getNormal(forceDirection);
                frictionDirection = (dot(fullVelocity, frictionDirection) > 0) ? -frictionDirection : frictionDirection;
                this->AddForceAt(pos, frictionDirection*this->mass*abs(dot(normalize(fullVelocity), frictionDirection))*0.5f);
            }
        }
    }
    void Update()
    {
        this->position += this->velocity * deltaTime;
        this->rotation += this->rotVelocity * deltaTime;

        this->velocity.y += 30.0f*deltaTime;

        // this->rotVelocity *= 0.99f;

        this->WallsCollide();
    }
};

class Collision {
public:
    int index1, index2;
    float penetration;
    Collision(int i1, int i2, float penetration) {
        this->index1 = i1;
        this->index2 = i2;
        this->penetration = penetration;
    }
};

vector<RigidBody> rigidBodies = vector<RigidBody>();
vector<Circle> Circles = vector<Circle>();

float maxFPS = 60;
Clock frameStartTime;
Text FPSDisplay;
Clock fpsDisplayUpdate;

Vector2i oldMousePosition;
Vector2i startMousePosition;
Vector2i mousePosition;
bool isMouseClicked;

bool isKeyboardClicked;
Keyboard::Key keyClicked;

CircleShape debugOnLinePoint;
Vertex debugDirection[2];
Text debugForceText;
float startClickPosition;
bool isReleased = true;

bool isPaused = false;

int main()
{
    cout << "Start" << endl
         << endl;
    ContextSettings settings;
    settings.antialiasingLevel = 1;
    RenderWindow window(VideoMode(WIDTH, HEIGHT), "SFML window", Style::Default, settings);
    window.setFramerateLimit(60);

    defaultFont = Font();
    defaultFont.loadFromFile("E:\\!Coding\\c++\\Projects\\SFML\\Default\\sansation.ttf");

    FPSDisplay.setFillColor(Color::Green);
    FPSDisplay.setFont(defaultFont);
    FPSDisplay.setCharacterSize(16);
    fpsDisplayUpdate = Clock();

    frameStartTime = Clock();

    VertexArray points(LinesStrip);
    for (int i = 0; i < 1; i++) {
        int pointsCount = 4;
        float S = 64.0f;
        for (int j = 0; j < pointsCount; j++)
        {
            float t = ((float)j / pointsCount) * 2 * PI;
            points.append(Vertex(Vector2f(cosf(t), sinf(t)) * (64.0f / sqrtf(2.0f)), Color::Red));
        }
        rigidBodies.push_back(RigidBody(points, Vector2f(100, HEIGHT - 32.0f), PI/4.0f, 1.0f, (S*S*2/12.0f)));

        float A = (-1.0f+random()*2.0f)*2*PI;
        float M = (-1.0f+random()*2.0f)*1;
        rigidBodies[rigidBodies.size()-1].velocity = Vector2f(cosf(A),sinf(A))*M;

        rigidBodies[rigidBodies.size()-1].rotVelocity = (-1.0f+random()*2.0f)*1;
        // rigidBodies[rigidBodies.size()-1].AddForceAt(rigidBodies[rigidBodies.size()-1].position+Vector2f(-32.0f,32.0f),Vector2f(0,-25.6f));
    }
    
    for (int i = 0; i < 20; i++) {
        Circle C = Circle(20.0f);
        C.position = Vector2f(20.0f+(float)i*50.0f,HEIGHT-50.f);
        C.rotVelocity = 3.0f;
        C.restitution = 0.3f;
        Circles.push_back(C);
    }

    // rigidBodies[0].AddForceAt(rigidBodies[0].position + Vector2f(0.5f, 0) * (128.0f / sqrtf(2.0f)), Vector2f(0, -1));
    // rigidBodies[0].AddForceAt(rigidBodies[0].position + Vector2f(-0.5f, 0) * (128.0f / sqrtf(2.0f)), Vector2f(0, 1));

    debugOnLinePoint.setRadius(2);
    debugOnLinePoint.setFillColor(Color::Red);

    debugForceText.setFillColor(Color::Green);
    debugForceText.setFont(defaultFont);
    debugForceText.setCharacterSize(8);

    while (window.isOpen())
    {
        WIDTH = window.getSize().x;
        HEIGHT = window.getSize().y;
        deltaTime = frameStartTime.getElapsedTime().asSeconds();
        // if (deltaTime < 1 / maxFPS)
        //     continue;
        frameStartTime.restart();

        if (fpsDisplayUpdate.getElapsedTime().asSeconds() > 0.12f)
        {
            FPSDisplay.setString(to_string((int)round(1 / deltaTime)));
            fpsDisplayUpdate.restart();
        }

        Event event;
        while (window.pollEvent(event))
        {
            if (event.type == Event::Closed)
                window.close();
            if (event.type == Event::Resized) {
                WIDTH = event.size.width;
                HEIGHT = event.size.height;
                window.setView(View(FloatRect(0,0,event.size.width,event.size.height)));
            }
            if (event.type == Event::MouseMoved)
            {
                mousePosition = Mouse::getPosition(window);
            }
            if (event.type == Event::MouseButtonPressed)
            {
                startMousePosition = mousePosition;
                isReleased = false;
                if (event.mouseButton.button == Mouse::Button::Middle)
                    isPaused = !isPaused;
            }
            if (event.type == Event::MouseButtonReleased)
            {
                isReleased = true;
                for (int i = 0; i < rigidBodies.size(); i++)
                {
                    rigidBodies[i].AddForceAt((Vector2f)startMousePosition, (Vector2f)(mousePosition - startMousePosition) / 80.0f*rigidBodies[i].mass/deltaTime);
                }
                for (int i = 0; i < Circles.size(); i++)
                {
                    float magn = magnitude(Circles[i].position-(Vector2f)startMousePosition);
                    Circles[i].AddForce((Vector2f)startMousePosition, (Vector2f)(mousePosition - startMousePosition) / 1.0f*Circles[i].mass/deltaTime/(1.0f+magn));
                }
            }
        }

        if (!isReleased)
        {
            // debugDirection[0].position = lines[0].position + lines[0].getDirection() * startClickPosition * lines[0].length;
            debugDirection[0].position = (Vector2f)startMousePosition;
            debugDirection[0].color = Color::Blue;

            debugDirection[1].position = (Vector2f)mousePosition;
            debugDirection[1].color = Color::Blue;

            debugForceText.setPosition((Vector2f)mousePosition);
            debugForceText.setString(to_string(magnitude((debugDirection[1].position - debugDirection[0].position) / 100.0f)));

            window.draw(debugForceText);
            window.draw(debugDirection, 2, Lines);
        }

        // debugOnLinePoint.setPosition(lines[0].position + lines[0].getDirection() * lines[0].getPositionOnLine((Vector2f)mousePosition) * lines[0].length);

        // window.draw(debugOnLinePoint);

        for (int i = 0; i < rigidBodies.size(); i++)
        {
            if (!isPaused)
                rigidBodies[i].Update();
            rigidBodies[i].Draw(&window);
        }
        vector<Collision> collisions;
        for (int i = 0; i < Circles.size(); i++) {
            for (int j = 0; j < Circles.size(); j++) {
                if (i==j) continue;
                float magn = magnitude(Circles[i].position-Circles[j].position);
                if (magn < Circles[i].radius+Circles[j].radius) {
                    bool isExist = false;
                    for (int k = 0; k < collisions.size(); k++) {
                        if ((collisions[k].index1 == i && collisions[k].index2 == j) ||
                            (collisions[k].index2 == i && collisions[k].index1 == j)) {

                            isExist = true;
                            break;
                        }
                    }
                    if (!isExist) {
                        collisions.push_back(Collision(i,j,Circles[i].radius+Circles[j].radius-magn));
                    }
                }
            }
        }
        for (int i = 0; i < collisions.size(); i++) {
            Circles[collisions[i].index1].CirclesCollision(Circles[collisions[i].index2].position,collisions[i].penetration);
            Circles[collisions[i].index2].CirclesCollision(Circles[collisions[i].index1].position,collisions[i].penetration);
        }

        for (int i = 0; i < Circles.size(); i++) {
            if (!isPaused)
                Circles[i].Update();
            Circles[i].Draw(&window);
        }

        window.draw(FPSDisplay);
        window.display();
        window.clear();
        frame++;
    }
    return 0;
}