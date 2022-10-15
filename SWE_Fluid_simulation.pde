String windowTitle = "SWE Fluid Simulation";

static int n = 30;
float dx = 500.0/n;

float h[] = new float[n];
float hu[] = new float[n];

//Midpoint helpers
float dhdt[] = new float[n]; //Height(Midpoint)
float dhudt[] = new float[n]; //Momentum(Midpoint)

void setup() {
  size(500, 400, P3D);
  surface.setTitle(windowTitle);
  initScene();
}

void initScene() {

}