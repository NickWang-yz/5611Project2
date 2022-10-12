//PDEs and Integration
//CSCI 5611 Swinging Rope [Exercise]
//Stephen J. Guy <sjguy@umn.edu>

//NOTE: The simulation starts paused, press "space" to unpause

//TODO:
//  1. The rope moves very slowly now, this is because the timestep is 1/20 of realtime
//      a. Make the timestep realtime (20 times faster than the inital code), what happens?
//      b. Call the small 1/20th timestep update, 20 times each frame (in a for loop) -- why is it different?
//  2. When the rope hanging down fully the spacing between the links is not equal, even though they
//     where initalized with an even spacing between each node. What is this?
//      - If this is a bug, fix the corisponding code
//      - If this why is not a bug, explain why this is the expected behavior
//  3. By default, the rope starts vertically. Change initScene() so it starts at an angle. The rope should
//     then swing backc and forth.
//  4. Try changing the mass and the k value. How do they interact wich each other?
//  5. Set the kv value very low, does the rope bounce a lot? What about a high kv (not too high)?
//     Why doesn't the rope stop swinging at high values of kv?
//  6. Add friction/drag so that the rope eventually stops. An easy friction model is a scaled force 
//     in the opposite direction of a nodes current velocity. 

//Challenge:
//  - Set the top of the rope to be wherever the user's mouse is, and allow the user to drag the rope around the scene.
//  - Keep the top of the rope fixed, but allow the user to click and drag one of the balls of the rope to move it around.
//  - Place a medium-sized, static 2D ball in the scene, have the nodes on the rope experience a "bounce" force if they collide with this ball.


//Create Window
Camera camera;

String windowTitle = "Swinging Rope";
void setup() {
  size(600, 600, P3D);

  camera = new Camera();

  camera.position = new PVector(200,200,500);

  surface.setTitle(windowTitle);
  initScene();
}

void keyReleased()
{
  camera.HandleKeyReleased();
}

void mousePressed(){
  camera.mousePressed(); 
}

void mouseReleased(){
    camera.mouseReleased(); 
}

void mouseDragged(){
   camera.mouseDragged(); 
}

void mouseWheel(MouseEvent event){
  camera.mouseWheel(event);
}

//Simulation Parameters
float floor = 500;
Vec2 gravity = new Vec2(0,400);
float radius = 5;
Vec2 obsticle = new Vec2(180,180);
float radiusObsticle = 20;
Vec2 stringTop = new Vec2(200,50);
float restLen = 10;
float mass = 1.0; //TRY-IT: How does changing mass affect resting length of the rope?
float k = 200; //TRY-IT: How does changing k affect resting length of the rope?
float kv = 30; //TRY-IT: How big can you make kv?

//Initial positions and velocities of masses
static int maxNodes = 100;
static int maxRope = 20;
Vec2 pos[][] = new Vec2[maxRope][maxNodes];
Vec2 vel[][] = new Vec2[maxRope][maxNodes];
Vec2 acc[][] = new Vec2[maxRope][maxNodes];

int numNodes = 10;
int numRopes = 10;

void initScene(){
  for(int j = 0; j < numRopes; j++) {
    for (int i = 0; i < numNodes; i++){
      pos[j][i] = new Vec2(0,0);
      pos[j][i].x = stringTop.x-50+10*j-20*i;
      pos[j][i].y = stringTop.y + 8*i; //Make each node a little lower
      vel[j][i] = new Vec2(0,0);
    }
  }
}

void update(float dt){

  //Reset accelerations each timestep (momenum only applies to velocity)
  for(int j = 0; j < numRopes; j++) {
    for (int i = 0; i < numNodes; i++){
      acc[j][i] = new Vec2(0,0);
      acc[j][i].add(gravity);
    }
  }
  
  for(int j = 0; j < numRopes-1; j++) {
    for(int i = 0; i < numNodes; i++) {
      Vec2 diff = pos[j+1][i].minus(pos[j][i]);
      float stringF = -k*(diff.length() - restLen);

      Vec2 stringDir = diff.normalized();
      float projVbot = dot(vel[j][i], stringDir);
      float projVtop = dot(vel[j+1][i], stringDir);
      float dampF = -kv*(projVtop - projVbot);

      Vec2 force = stringDir.times(stringF+dampF);
      acc[j][i].add(force.times(-1.0/mass));
      acc[j+1][i].add(force.times(1.0/mass));
    }
  }

  //Compute (damped) Hooke's law for each spring
  for(int j = 0; j < numRopes; j++) {
    for (int i = 0; i < numNodes-1; i++){
      Vec2 diff = pos[j][i+1].minus(pos[j][i]);
      float stringF = -k*(diff.length() - restLen);
      //println(stringF,diff.length(),restLen);
      
      Vec2 stringDir = diff.normalized();
      float projVbot = dot(vel[j][i], stringDir);
      float projVtop = dot(vel[j][i+1], stringDir);
      float dampF = -kv*(projVtop - projVbot);
      
      Vec2 force = stringDir.times(stringF+dampF);
      acc[j][i].add(force.times(-1.0/mass));
      acc[j][i+1].add(force.times(1.0/mass));
      
    }
  }

  //Eulerian integration
  for(int j = 0; j < numRopes; j++) {
    for (int i = 1; i < numNodes; i++){
      vel[j][i].add(acc[j][i].times(dt));
      pos[j][i].add(vel[j][i].times(dt));
    }
  }
  
  //Collision detection and response
  for(int j = 0; j < numRopes; j++) {
    for (int i = 0; i < numNodes; i++){
      if (pos[j][i].y+radius > floor){
        vel[j][i].y *= -.9;
        pos[j][i].y = floor - radius;
      }
    }
  }

  for(int j = 0; j < numRopes; j++) {
    for(int i = 0; i < numNodes; i++) {
      float d = pos[j][i].distanceTo(obsticle);
      if(d < radiusObsticle +0.09) {
        Vec2 n = pos[j][i].minus(obsticle).normalized();
        float lengthInDirection = dot(n, vel[j][i]);
        Vec2 bounce = n.times(lengthInDirection);
        vel[j][i].subtract(bounce.times(1.5));
        pos[j][i].add(n.times(0.1+radiusObsticle-d));
      }
    }
  }
  
}

// Draws a scaled, textured quad at the given position.
void drawTexturedQuad(PVector position, float scale, PImage texture)
{
  pushMatrix();
  translate(position.x, position.y, position.z);
  scale(scale, scale, scale);
  beginShape();
  texture(texture);
  vertex(-1, -1, 0, 0, 0);
  vertex(1, -1, 0, texture.width, 0);
  vertex(1, 1, 0, texture.width, texture.height);
  vertex(-1, 1, 0, 0, texture.height);
  endShape();
  popMatrix();
} 

//Draw the scene: one sphere per mass, one line connecting each pair
boolean paused = true;
void draw() {
  background(255);
  camera.update(1.0/frameRate);

  directionalLight(255.0, 255.0, 255.0, 0, -1, -1);

  // fill(20,200,150);
  // for(int j = 0; j < numRopes; j++) {
  //   for(int i = 0; i < numNodes; i++) {
  //     circle(pos[j][i].x, pos[j][i].y, radius*2);
  //   }
  // }


  for(int i = 0; i < 20; i++)
  {
    if (!paused) update(1/(20*frameRate));
  }
 
  fill(20,200,150);

  for(int j = 0; j < numRopes; j++) {
    for (int i = 0; i < numNodes-1; i++){
      pushMatrix();
      line(pos[j][i].x,pos[j][i].y,pos[j][i+1].x,pos[j][i+1].y);
      translate(pos[j][i+1].x,pos[j][i+1].y);
      sphere(radius);
      popMatrix();
    }
  }

  for(int j = 0; j < numRopes-1; j++) {
    for (int i = 0; i < numNodes; i++){
      pushMatrix();
      line(pos[j][i].x,pos[j][i].y,pos[j+1][i].x,pos[j+1][i].y);
      popMatrix();
    }
  }
  
  
  
  pushMatrix();
  translate(obsticle.x,obsticle.y);
  fill(255,0,0);
  sphere(radiusObsticle);
  popMatrix();

  if (paused)
    surface.setTitle(windowTitle + " [PAUSED]");
  else
    surface.setTitle(windowTitle + " "+ nf(frameRate,0,2) + "FPS");
}

void keyPressed(){
  if (key == ' ') {
    paused = !paused;
  }
  camera.HandleKeyPressed();
}


///////////////////
// Vec2D Library
///////////////////

// public class Vec2 {
//   public float x, y;
  
//   public Vec2(float x, float y){
//     this.x = x;
//     this.y = y;
//   }
  
//   public String toString(){
//     return "(" + x+ ", " + y +")";
//   }
  
//   public float length(){
//     return sqrt(x*x+y*y);
//   }
  
//   public float lengthSqr(){
//     return x*x+y*y;
//   }
  
//   public Vec2 plus(Vec2 rhs){
//     return new Vec2(x+rhs.x, y+rhs.y);
//   }
  
//   public void add(Vec2 rhs){
//     x += rhs.x;
//     y += rhs.y;
//   }
  
//   public Vec2 minus(Vec2 rhs){
//     return new Vec2(x-rhs.x, y-rhs.y);
//   }
  
//   public void subtract(Vec2 rhs){
//     x -= rhs.x;
//     y -= rhs.y;
//   }
  
//   public Vec2 times(float rhs){
//     return new Vec2(x*rhs, y*rhs);
//   }
  
//   public void mul(float rhs){
//     x *= rhs;
//     y *= rhs;
//   }
  
//   public void normalize(){
//     float magnitude = sqrt(x*x + y*y);
//     x /= magnitude;
//     y /= magnitude;
//   }
  
//   public Vec2 normalized(){
//     float magnitude = sqrt(x*x + y*y);
//     return new Vec2(x/magnitude, y/magnitude);
//   }
  
//   public void clampToLength(float maxL){
//     float magnitude = sqrt(x*x + y*y);
//     if (magnitude > maxL){
//       x *= maxL/magnitude;
//       y *= maxL/magnitude;
//     }
//   }
  
//   public void setToLength(float newL){
//     float magnitude = sqrt(x*x + y*y);
//     x *= newL/magnitude;
//     y *= newL/magnitude;
//   }
  
//   public float distanceTo(Vec2 rhs){
//     float dx = rhs.x - x;
//     float dy = rhs.y - y;
//     return sqrt(dx*dx + dy*dy);
//   }
  
// }

// Vec2 interpolate(Vec2 a, Vec2 b, float t){
//   return a.plus((b.minus(a)).times(t));
// }

// float interpolate(float a, float b, float t){
//   return a + ((b-a)*t);
// }

// float dot(Vec2 a, Vec2 b){
//   return a.x*b.x + a.y*b.y;
// }

// Vec2 projAB(Vec2 a, Vec2 b){
//   return b.times(a.x*b.x + a.y*b.y);
// }
