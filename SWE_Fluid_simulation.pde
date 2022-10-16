String windowTitle = "SWE Fluid Simulation";

static int n = 30;
float dx = 500.0/n;
float g = 400;
float damp = 0.9;
float dt = 0.01;
float sim_dt = 0.001;

float h[] = new float[n];
float hu[] = new float[n];
float h_mid[] = new float[n];
float hu_mid[] = new float[n];

//Midpoint helpers
float dhdt[] = new float[n]; //Height(Midpoint)
float dhudt[] = new float[n]; //Momentum(Midpoint)
float dhdt_mid[] = new float[n];
float dhudt_mid[] = new float[n];

int rect_x[] = new int[n];
int rect_y[] = new int[n];

void setup() {
  size(600, 400, P3D);
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
        rect_y[i] = 400;
    }

    for(int i = 10; i < 20; i++) {
        h[i] = 200-i;
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

    for(int i = 0; i < n-1; i++) {
        h[i] += damp*dhdt[i]*dt;
        hu[i] += damp*dhudt[i]*dt;
    }

    h[0] = h[n-2];
    h[n-1] = h[1];

    hu[0] = hu[n-2];
    hu[n-1] = hu[1];
}

boolean paused = true;
void draw() {
    background(255);

    fill(0, 0, 255);

    for (int i = 0; i < int(dt/sim_dt); i++) {
        if (!paused) update(dt);
    }

    for(int i = 0; i < n; i++) {
        rect(rect_x[i], rect_y[i], 600/n, -h[i]);
        
    }

    if (paused)
        surface.setTitle(windowTitle + " [PAUSED]");
    else
        surface.setTitle(windowTitle + " "+ nf(frameRate,0,2) + "FPS");
}

void keyPressed(){
  if (key == ' ') {
    paused = !paused;
  }
}