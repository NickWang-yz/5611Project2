String windowTitle = "SWE Fluid Simulation";

static int n = 50;
float dx = 500.0/n;
float g = 400;
float damp = 0.9;
float dt = 0.01;
float sim_dt = 0.005;

float h[] = new float[n];
float hu[] = new float[n];
float h_mid[] = new float[n];
float hu_mid[] = new float[n];

//Midpoint helpers
float dhdt[] = new float[n]; //Height(Midpoint)
float dhudt[] = new float[n]; //Momentum(Midpoint)
float dhdt_mid[] = new float[n];
float dhudt_mid[] = new float[n];

float rect_x[] = new float[n];
float rect_y[] = new float[n];

boolean periodic = true;
boolean free = false;
boolean reflective = false;

Camera camera = new Camera();;

void setup() {
  size(600, 400, P3D);

  camera.position = new PVector(312,422,498);
  camera.phi = 1;

  surface.setTitle(windowTitle);
  initScene();
}

void initScene() {
    for(int i = 0; i < n; i++) {
        h[i] = 100;
        hu[i] = 0;
        dhdt[i] = 0;
        dhudt[i] = 0;
        rect_x[i] = (600/n)*i;
    }

    for(int i = 1; i < 10; i++) {
        h[i] = 100+10*i;
    }

    for(int i = 10; i < 20; i++) {
        h[i] = 200-10*(i-10);
    }
    
    for(int i = 0; i < n; i++) {
      rect_y[i] = (h[i]+100)/2;
    }
}

void update(float dt) {
    for(int i = 0; i < n-1; i++) {
        h_mid[i] = (h[i]+h[i+1])/2;
        hu_mid[i] = (hu[i]+hu[i+1])/2;

        float dhudx_mid = (hu[i+1]-hu[i])/dx;
        dhdt_mid[i] = -dhudx_mid;

        float dhu2dx_mid = ((hu[i+1]*hu[i+1])/h[i+1] - (hu[i]*hu[i])/h[i])/dx;
        float dgh2dx_mid = g*((h[i+1]*h[i+1]) - (h[i]*h[i]))/dx;

        dhudt_mid[i] = -(dhu2dx_mid + 0.5*dgh2dx_mid);

        h_mid[i] += dhdt_mid[i]*dt/2;
        hu_mid[i] += dhudt_mid[i]*dt/2;
    }

    for(int i = 1; i < n-1; i++) {
        float dhudx = (hu_mid[i] - hu_mid[i-1])/dx;
        dhdt[i] = -dhudx;

        float dhu2dx = ((hu_mid[i]*hu_mid[i])/h_mid[i] - (hu_mid[i-1]*hu_mid[i-1])/h_mid[i-1])/dx;
        float dgh2dx = g*((h_mid[i]*h_mid[i]) - (h_mid[i-1]*h_mid[i-1]))/dx;
        dhudt[i] = -(dhu2dx + .5*dgh2dx);
    }

    for(int i = 0; i < n; i++) {
        h[i] += damp*dhdt[i]*dt;
        hu[i] += damp*dhudt[i]*dt;
        rect_y[i] = (h[i]+100)/2;
    }

    if(periodic) {
        //periodic
        h[0] = h[n-2];
        h[n-1] = h[1];

        hu[0] = hu[n-2];
        hu[n-1] = hu[1];
    }
    else if(free) {
        //free
        h[0] = h[1];
        h[n-1] = h[n-2];

        hu[0] = hu[1];
        hu[n-1] = hu[n-2];
    }
    else {
        //reflective
        h[0] = h[1];
        h[n-1] = h[n-2];

        hu[0] = -hu[1];
        hu[n-1] = -hu[n-2];
    }
        
}

boolean paused = true;
void draw() {
    background(255);
    camera.update(1.0/frameRate);

    directionalLight(255.0, 255.0, 255.0, 0, -1, -1);

    

    for (int i = 0; i < int(dt/sim_dt); i++) {
        if (!paused) update(dt);
    }

    fill(220,220,220);
    pushMatrix();
    translate(rect_x[25]-6, -250, 200);
    box(600, 10, 400);
    popMatrix();

    fill(220,220,220);
    pushMatrix();
    translate(0, 0, 200);
    box(10, 500, 400);
    popMatrix();

    fill(220,220,220);
    pushMatrix();
    translate(rect_x[25]-6, 250, 200);
    box(600, 10, 400);
    popMatrix();

    fill(220,220,220);
    pushMatrix();
    translate(rect_x[49], 0, 200);
    box(10, 500, 400);
    popMatrix();

    fill(0, 0, 255);    
    for(int i = 0; i < n; i++) {
      pushMatrix();  
      translate(rect_x[i], 0, rect_y[i]);
      box(600/n, 500, h[i]);
      popMatrix();
    }
    //println(camera.theta, " ", camera.phi);

    if (paused)
        surface.setTitle(windowTitle + " [PAUSED]");
    else
        surface.setTitle(windowTitle + " "+ nf(frameRate,0,2) + "FPS");
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

void keyPressed(){
  if (key == ' ') {
    paused = !paused;
  }

  if(key == '1') {
    periodic = true;
    free = false;
    reflective = false;
  }

  if(key == '2') {
    periodic = false;
    free = true;
    reflective = false;
  }

  if(key == '3') {
    periodic = false;
    free = false;
    reflective = true;
  }

  camera.HandleKeyPressed();
}