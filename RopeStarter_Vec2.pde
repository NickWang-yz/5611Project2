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

  camera.position = new PVector(158,100,170);

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
PVector gravity = new PVector(0,400.0,0);
float radius = 1;
PVector obsticle = new PVector(180,90,10);
float radiusObsticle = 25;
PVector stringTop = new PVector(200,50,0);
float restLen = 1;
float mass = 1.0; //TRY-IT: How does changing mass affect resting length of the rope?
float k = 250; //TRY-IT: How does changing k affect resting length of the rope?
float kv = 30; //TRY-IT: How big can you make kv?

//Initial positions and velocities of masses
static int maxNodes = 100;
static int maxRope = 100;
PVector pos[][] = new PVector[maxRope][maxNodes];
PVector vel[][] = new PVector[maxRope][maxNodes];
PVector acc[][] = new PVector[maxRope][maxNodes];

int numNodes = 10;
int numRopes = 50;
int numNodes2 =0;
PVector windDirection = new PVector(0, 0, 1);
float windMagnitude = 0;
boolean windBreak = false;

void initScene(){
  for(int j = 0; j < numRopes; j++) {
    for (int i = 0; i < numNodes; i++){
      pos[j][i] = new PVector(0,50,0);
      pos[j][i].x = (stringTop.x -50 + restLen*j)+8;
      pos[j][i].z = (stringTop.y + 8*i)-90; //Make each node a little lower
      vel[j][i] = new PVector(0,0,0);
    }
  }
}

void update(float dt){
  //Reset accelerations each timestep (momenum only applies to velocity)
  for(int j = 0; j < numRopes; j++) {
    for (int i = 0; i < numNodes; i++){
      acc[j][i] = new PVector(0,0,0);
      acc[j][i].add(gravity);
    }
  }
  
  for(int j = 0; j < numRopes-1; j++) {
    for(int i = 0; i < numNodes; i++) {
      PVector diff = PVector.sub(pos[j+1][i], pos[j][i]);
      float stringF = -k*(diff.mag() - restLen);

      PVector stringDir = diff;
      stringDir.normalize();
      float projVbot = PVector.dot(vel[j][i], stringDir);
      float projVtop = PVector.dot(vel[j+1][i], stringDir);
      float dampF = -kv*1.2*(projVtop - projVbot);

      PVector force = PVector.mult(stringDir,(stringF+dampF));
      acc[j][i].add(PVector.mult(force,(-1.0/mass)));
      acc[j+1][i].add(PVector.mult(force,(1.0/mass)));
    }
  }

  //Compute (damped) Hooke's law for each spring
  for(int j = 0; j < numRopes; j++) {
    for (int i = 0; i < numNodes-1; i++){
      PVector diff = PVector.sub(pos[j][i+1],pos[j][i]);
      float stringF = -k*(diff.mag() - restLen);
      //println(stringF,diff.length(),restLen);
      
      PVector stringDir = diff;
      stringDir.normalize();
      float projVbot = PVector.dot(vel[j][i], stringDir);
      float projVtop = PVector.dot(vel[j][i+1], stringDir);
      float dampF = -kv*(projVtop - projVbot);
      
      PVector force = PVector.mult(stringDir,(stringF+dampF));
      acc[j][i].add(PVector.mult(force,(-1.0/mass)));
      acc[j][i+1].add(PVector.mult(force,(1.0/mass)));
      
    }
  }

  //Eulerian integration
  for(int j = 0; j < numRopes; j++) {
    for (int i = 1; i < numNodes; i++){
      acc[j][i].add(PVector.mult(windDirection, windMagnitude));
      vel[j][i].add(PVector.mult(acc[j][i],(dt)));
      vel[j][i].mult(0.9999); 
      
      pos[j][i].add(PVector.mult(vel[j][i],(dt)));
    }
  }

  for(int j = 0; j < numRopes; j++) {
    for (int i = 1; i < numNodes; i++){
      if(windMagnitude > 330){
        numNodes = 5;
        numNodes2 = 5;
        windBreak = true;
        break;
      }
    }
  }
  
  //Collision detection and response
  for(int j = 0; j < numRopes; j++) {
    for (int i = 0; i < numNodes; i++){
      if (pos[j][i].z+radius > floor){
        vel[j][i].y *= -.5;
        pos[j][i].y = floor - radius;
      }
    }
  }

  for(int j = 0; j < numRopes; j++) {
    for(int i = 0; i < numNodes; i++) {
      float d = pos[j][i].dist(obsticle);
      if(d < radiusObsticle +0.09) {
        PVector n = PVector.sub(pos[j][i],(obsticle));
        n.normalize();
        float lengthInDirection = PVector.dot(n, vel[j][i]);
        PVector bounce = PVector.mult(n,(lengthInDirection));
        vel[j][i].sub(PVector.mult(bounce,(1.1)));
        //pos[j][i] = PVector.mult(n, radiusObsticle+0.1);
        pos[j][i].add(PVector.mult(n,(0.1+radiusObsticle-d)));
      }
    }
  }

  //println("camera: ", camera.position.x, camera.position.y, camera.position.z);
  println("windmagnitude: ", windMagnitude);
}


void update2(float dt) {
  for(int j = 0; j < numRopes; j++) {
    for (int i = 0; i < numNodes; i++){
      acc[j][i] = new PVector(0,0,0);
      acc[j][i].add(gravity);
    }

    for(int i = 5; i < numNodes2; i++) {
      acc[j][i] = new PVector(0,0,0);
      acc[j][i].add(gravity);
    }
  }

  for(int j = 0; j < numRopes-1; j++) {
    for(int i = 0; i < numNodes; i++) {
      PVector diff = PVector.sub(pos[j+1][i], pos[j][i]);
      float stringF = -k*(diff.mag() - restLen);

      PVector stringDir = diff;
      stringDir.normalize();
      float projVbot = PVector.dot(vel[j][i], stringDir);
      float projVtop = PVector.dot(vel[j+1][i], stringDir);
      float dampF = -kv*1.2*(projVtop - projVbot);

      PVector force = PVector.mult(stringDir,(stringF+dampF));
      acc[j][i].add(PVector.mult(force,(-1.0/mass)));
      acc[j+1][i].add(PVector.mult(force,(1.0/mass)));
    }

    for(int i = 5; i < numNodes2; i++) {
      PVector diff = PVector.sub(pos[j+1][i], pos[j][i]);
      float stringF = -k*(diff.mag() - restLen);

      PVector stringDir = diff;
      stringDir.normalize();
      float projVbot = PVector.dot(vel[j][i], stringDir);
      float projVtop = PVector.dot(vel[j+1][i], stringDir);
      float dampF = -kv*1.2*(projVtop - projVbot);

      PVector force = PVector.mult(stringDir,(stringF+dampF));
      acc[j][i].add(PVector.mult(force,(-1.0/mass)));
      acc[j+1][i].add(PVector.mult(force,(1.0/mass)));
    }
  }

  for(int j = 0; j < numRopes; j++) {
    for (int i = 0; i < numNodes-1; i++){
      PVector diff = PVector.sub(pos[j][i+1],pos[j][i]);
      float stringF = -k*(diff.mag() - restLen);
      //println(stringF,diff.length(),restLen);
      
      PVector stringDir = diff;
      stringDir.normalize();
      float projVbot = PVector.dot(vel[j][i], stringDir);
      float projVtop = PVector.dot(vel[j][i+1], stringDir);
      float dampF = -kv*(projVtop - projVbot);
      
      PVector force = PVector.mult(stringDir,(stringF+dampF));
      acc[j][i].add(PVector.mult(force,(-1.0/mass)));
      acc[j][i+1].add(PVector.mult(force,(1.0/mass)));
    }

    for(int i = 5; i < numNodes2-1; i++) {
      PVector diff = PVector.sub(pos[j][i+1],pos[j][i]);
      float stringF = -k*(diff.mag() - restLen);
      //println(stringF,diff.length(),restLen);
      
      PVector stringDir = diff;
      stringDir.normalize();
      float projVbot = PVector.dot(vel[j][i], stringDir);
      float projVtop = PVector.dot(vel[j][i+1], stringDir);
      float dampF = -kv*(projVtop - projVbot);
      
      PVector force = PVector.mult(stringDir,(stringF+dampF));
      acc[j][i].add(PVector.mult(force,(-1.0/mass)));
      acc[j][i+1].add(PVector.mult(force,(1.0/mass)));
    }
  }

  //Eulerian integration
  for(int j = 0; j < numRopes; j++) {
    for (int i = 1; i < numNodes; i++){
      acc[j][i].add(PVector.mult(windDirection, windMagnitude));
      vel[j][i].add(PVector.mult(acc[j][i],(dt)));
      vel[j][i].mult(0.9999); 
      
      pos[j][i].add(PVector.mult(vel[j][i],(dt)));
    }

    for (int i = 5; i < numNodes2; i++){
      acc[j][i].add(PVector.mult(windDirection, windMagnitude));
      vel[j][i].add(PVector.mult(acc[j][i],(dt)));
      vel[j][i].mult(0.9999); 
      
      pos[j][i].add(PVector.mult(vel[j][i],(dt)));
    }
  }
  
  //Collision detection and response
  for(int j = 0; j < numRopes; j++) {
    for (int i = 0; i < numNodes; i++){
      if (pos[j][i].z+radius > floor){
        vel[j][i].y *= -.5;
        pos[j][i].y = floor - radius;
      }
    }

    for (int i = 5; i < numNodes2; i++){
      if (pos[j][i].z+radius > floor){
        vel[j][i].y *= -.5;
        pos[j][i].y = floor - radius;
      }
    }
  }

  for(int j = 0; j < numRopes; j++) {
    for(int i = 0; i < numNodes; i++) {
      float d = pos[j][i].dist(obsticle);
      if(d < radiusObsticle +0.09) {
        PVector n = PVector.sub(pos[j][i],(obsticle));
        n.normalize();
        float lengthInDirection = PVector.dot(n, vel[j][i]);
        PVector bounce = PVector.mult(n,(lengthInDirection));
        vel[j][i].sub(PVector.mult(bounce,(1.1)));
        //pos[j][i] = PVector.mult(n, radiusObsticle+0.1);
        pos[j][i].add(PVector.mult(n,(0.1+radiusObsticle-d)));
      }
    }

    for(int i = 5; i < numNodes2; i++) {
      float d = pos[j][i].dist(obsticle);
      if(d < radiusObsticle +0.09) {
        PVector n = PVector.sub(pos[j][i],(obsticle));
        n.normalize();
        float lengthInDirection = PVector.dot(n, vel[j][i]);
        PVector bounce = PVector.mult(n,(lengthInDirection));
        vel[j][i].sub(PVector.mult(bounce,(1.1)));
        //pos[j][i] = PVector.mult(n, radiusObsticle+0.1);
        pos[j][i].add(PVector.mult(n,(0.1+radiusObsticle-d)));
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

  for(int i = 0; i < 40; i++)
  {
    if (!paused) {
      if(!windBreak) {
        update(1/(40*frameRate));
      }
      else {
        update2(1/(40*frameRate));
      }
    }
     
  }
 
  fill(20,200,150);

  // for(int j = 0; j < numRopes; j++) {
  //   for (int i = 0; i < numNodes-1; i++){
  //     pushMatrix();
  //     line(pos[j][i].x,pos[j][i].y,pos[j][i].z,pos[j][i+1].x,pos[j][i+1].y,pos[j][i+1].z);
  //     translate(pos[j][i+1].x,pos[j][i+1].y,pos[j][i+1].z);
  //     sphere(radius);
  //     popMatrix();
  //   }
  // }

  // for(int j = 0; j < numRopes-1; j++) {
  //   for (int i = 0; i < numNodes; i++){
  //     pushMatrix();
  //     line(pos[j][i].x,pos[j][i].y,pos[j][i].z,pos[j+1][i].x,pos[j+1][i].y,pos[j+1][i].z);
  //     popMatrix();
  //   }
  // }


  fill(0,255,0);
  strokeWeight(0);
  for(int j = 0; j < numRopes-1; j++) {
    for(int i = 0; i < numNodes-1; i++) {
      beginShape();
      vertex(pos[j][i].x, pos[j][i].y, pos[j][i].z);
      vertex(pos[j+1][i].x, pos[j+1][i].y, pos[j+1][i].z);
      vertex(pos[j][i+1].x, pos[j][i+1].y, pos[j][i+1].z);
      endShape();

      beginShape();
      vertex(pos[j+1][i].x, pos[j+1][i].y, pos[j+1][i].z);
      vertex(pos[j+1][i+1].x, pos[j+1][i+1].y, pos[j+1][i+1].z);
      vertex(pos[j][i+1].x, pos[j][i+1].y, pos[j][i+1].z);
      endShape();
    }
  }
  
  
  
  pushMatrix();
  translate(obsticle.x,obsticle.y,obsticle.z);
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

  if(key == 'h') {
    // if(windMagnitude < 210) {
    //   windMagnitude+=10;
    // }
    // else {
    //   println("Woops, the wind can't be larger!");
    // }
    windMagnitude+=10;

    
  }
  camera.HandleKeyPressed();
}
