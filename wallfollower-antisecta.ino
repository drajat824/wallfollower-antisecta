#include "fis_header.h"
#include <NewPing.h>
#include <MechaQMC5883.h>

// QMC MAGNETOMETER

MechaQMC5883 qmc;
// Nilai maksimum dan minimum dari hasil kalibrasi
int x_max = -1415, x_min = -3491;
int y_max = 97, y_min = -1747;
int z_max = 10, z_min = -2216;
// Hitung offset
int x_offset = (x_max + x_min) / 2;
int y_offset = (y_max + y_min) / 2;
int z_offset = (z_max + z_min) / 2;
// Hitung skala
float x_scale = 2.0 / (x_max - x_min);
float y_scale = 2.0 / (y_max - y_min);
float z_scale = 2.0 / (z_max - z_min);

#define raspiPin 34

// Magnetometer
int initialGroup = -1;
int currentGroup = 0;
// *keterangan : 0 = kanan, 1 = kiri
bool sensorSide = 0;

// Number of inputs & outputs
const int fis_gcI = 1;
const int fis_gcO = 2;
const int fis_gcR = 4;

FIS_TYPE g_fisInput[fis_gcI];
FIS_TYPE g_fisOutput[fis_gcO];

// PWM Settings
const int freq = 50000;
const int pwmChannelLeft = 0;
const int pwmChannelRight = 1;
const int resolution = 8;

// Motor Pin Definitions
#define M2RightCycle 25
#define MRight1 12
#define MRight2 14
#define MLeft1 27
#define MLeft2 26
#define M2LeftCycle 13
int leftCycle = 255;
int rightCycle = 255;

//Ultrasonic
#define US1_ECHO 15
#define US1_TRIGGER 2

#define US2_TRIGGER 5
#define US2_ECHO 4

#define US3_TRIGGER 19
#define US3_ECHO 18

// Ultrasonic Sensor Instances
NewPing sonar1(US1_TRIGGER, US1_ECHO);
NewPing sonar2(US2_TRIGGER, US2_ECHO);
NewPing sonar3(US3_TRIGGER, US3_ECHO);

// Distance Variables
int jarakKanan, jarakDepan, jarakKiri;

// Variabel untuk jeda waktu
unsigned long previousMillis = 0;
const long interval = 2000;  // Jeda 5 detik

void setup() {
  Serial.begin(115200);
  // Serial2.begin(9600);

  //Raspberry Data
  pinMode(raspiPin, INPUT_PULLDOWN);

  // Magnetometer
  Wire.begin();
  qmc.init();

  // Motor Pin Configuration
  pinMode(MLeft1, OUTPUT);
  pinMode(MLeft2, OUTPUT);
  pinMode(M2LeftCycle, OUTPUT);
  pinMode(MRight1, OUTPUT);
  pinMode(MRight2, OUTPUT);
  pinMode(M2RightCycle, OUTPUT);

  ledcSetup(pwmChannelLeft, freq, resolution);
  ledcSetup(pwmChannelRight, freq, resolution);
  ledcAttachPin(M2LeftCycle, pwmChannelLeft);
  ledcAttachPin(M2RightCycle, pwmChannelRight);

  delay(500);
}

void loop() {

  // MAGNETOMETER
  int x, y, z;
  float azimuth;
  int bearing;
  qmc.read(&x, &y, &z);
  float x_comp = (x - x_offset) * x_scale;
  float y_comp = (y - y_offset) * y_scale;
  azimuth = atan2(y_comp, x_comp) * 180.0 / PI;
  if (azimuth < 0) {
    azimuth += 360.0;
  }
  bearing = (int)((azimuth + 11.25) / 22.5);
  if (bearing == 0) {
    bearing = 16;
  }
  int currentGroup = getAzimuthGroup(bearing);
  if (initialGroup == -1) {
    initialGroup = currentGroup;
  }
  unsigned long currentMillis = millis();
  if (isOppositeGroup(initialGroup, currentGroup)) {
    if (currentMillis - previousMillis >= interval) {
      toggleSensorSide();
      initialGroup = currentGroup;
      previousMillis = currentMillis;
    }
  }
  
  int jKn = sonar1.ping_median();
  int jD = sonar2.ping_median();
  int jKr = sonar3.ping_median();

  jarakKanan = sonar1.convert_cm(jKn);
  jarakDepan = sonar2.convert_cm(jD);
  jarakKiri = sonar3.convert_cm(jKr);
  delay(50);

  Serial.print("Jarak Kanan: ");
  Serial.println(jarakKanan);
  Serial.print("Jarak Depan: ");
  Serial.println(jarakDepan);
  Serial.print("Jarak Kiri: ");
  Serial.println(jarakKiri);
  Serial.println();

  // INPUT FUZZY
  if (sensorSide == 0) {
    g_fisInput[0] = jarakKanan;
  } else {
    g_fisInput[0] = jarakKiri;
  }

  // OUTPUT FUZZY
  g_fisOutput[0] = 0;
  g_fisOutput[1] = 0;

  // CEK RASPBERRY
  int raspi = 0;
  raspi = digitalRead(raspiPin);

  if (raspi) {
    moveStop();
    Serial.println("Mendeteksi Hama..");
    delay(4000);
  } else if (jarakDepan <= 10) {
    moveStop();
    Serial.println("Berhenti.");
  } else {
    fis_evaluate();

    if (sensorSide == 0) {
      // Sensor Kiri
      leftCycle = constrain((255 * g_fisOutput[0]), 0, 255);
      rightCycle = constrain((255 * g_fisOutput[1]), 0, 255);
    } else {
      // Sensor Kanan
      leftCycle = constrain((255 * g_fisOutput[1]), 0, 255);
      rightCycle = constrain((255 * g_fisOutput[0]), 0, 255);
    }

    moveForward();
    ledcWrite(pwmChannelLeft, leftCycle);
    ledcWrite(pwmChannelRight, rightCycle);

    Serial.print("Kecepatan Kanan: ");
    Serial.println(rightCycle);
    Serial.print("Kecepatan Kiri: ");
    Serial.println(leftCycle);
    Serial.println();
  }
}

int getAzimuthGroup(int bearing) {
  if (bearing >= 15 || bearing <= 2) {
    return 1;
  } else if (bearing >= 3 && bearing <= 6) {
    return 2;
  } else if (bearing >= 7 && bearing <= 10) {
    return 3;
  } else if (bearing >= 11 && bearing <= 14) {
    return 4;
  }
  return -1;  // Nilai default jika terjadi kesalahan
}

bool isOppositeGroup(int initialGroup, int currentGroup) {
  return (initialGroup == 1 && currentGroup == 3) || (initialGroup == 3 && currentGroup == 1) || (initialGroup == 2 && currentGroup == 4) || (initialGroup == 4 && currentGroup == 2);
}

void toggleSensorSide() {
  if (sensorSide == 0) {
    sensorSide = 1;
  } else {
    sensorSide = 0;
  }
}

void moveForward() {
  digitalWrite(MLeft1, HIGH);
  digitalWrite(MLeft2, LOW);
  digitalWrite(MRight1, HIGH);
  digitalWrite(MRight2, LOW);
}

void moveStop() {
  digitalWrite(MLeft1, LOW);
  digitalWrite(MLeft2, LOW);
  digitalWrite(MRight1, LOW);
  digitalWrite(MRight2, LOW);
}

// Fuyzz MATLAB

FIS_TYPE fis_trapmf(FIS_TYPE x, FIS_TYPE* p) {
  FIS_TYPE a = p[0], b = p[1], c = p[2], d = p[3];
  FIS_TYPE t1 = ((x <= c) ? 1 : ((d < x) ? 0 : ((c != d) ? ((d - x) / (d - c)) : 0)));
  FIS_TYPE t2 = ((b <= x) ? 1 : ((x < a) ? 0 : ((a != b) ? ((x - a) / (b - a)) : 0)));
  return (FIS_TYPE)min(t1, t2);
}

FIS_TYPE fis_min(FIS_TYPE a, FIS_TYPE b) {
  return min(a, b);
}

FIS_TYPE fis_max(FIS_TYPE a, FIS_TYPE b) {
  return max(a, b);
}

FIS_TYPE fis_array_operation(FIS_TYPE* array, int size, _FIS_ARR_OP pfnOp) {
  int i;
  FIS_TYPE ret = 0;

  if (size == 0) return ret;
  if (size == 1) return array[0];

  ret = array[0];
  for (i = 1; i < size; i++) {
    ret = (*pfnOp)(ret, array[i]);
  }

  return ret;
}


//*************************
// Data for Fuzzy Inference System
//*************************
// Pointers to the implementations of member functions
_FIS_MF fis_gMF[] = {
  fis_trapmf
};

// Count of member function for each Input
int fis_gIMFCount[] = { 4 };

// Count of member function for each Output
int fis_gOMFCount[] = { 2, 2 };

// Coefficients for the Input Member Functions
FIS_TYPE fis_gMFI0Coeff1[] = { 0, 0, 20, 30 };
FIS_TYPE fis_gMFI0Coeff2[] = { 20, 30, 30, 40 };
FIS_TYPE fis_gMFI0Coeff3[] = { 30, 40, 50, 55 };
FIS_TYPE fis_gMFI0Coeff4[] = { 50, 60, 70, 99999999 };
FIS_TYPE* fis_gMFI0Coeff[] = { fis_gMFI0Coeff1, fis_gMFI0Coeff2, fis_gMFI0Coeff3, fis_gMFI0Coeff4 };
FIS_TYPE** fis_gMFICoeff[] = { fis_gMFI0Coeff };

// Coefficients for the Output Member Functions
FIS_TYPE fis_gMFO0Coeff1[] = { 0, 0, 0.39, 0.91 };
FIS_TYPE fis_gMFO0Coeff2[] = { 0.78, 1.04, 1.3, 1.3 };
FIS_TYPE* fis_gMFO0Coeff[] = { fis_gMFO0Coeff1, fis_gMFO0Coeff2 };
FIS_TYPE fis_gMFO1Coeff1[] = { 0, 0, 0.39, 0.91 };
FIS_TYPE fis_gMFO1Coeff2[] = { 0.78, 1.04, 1.3, 1.3 };
FIS_TYPE* fis_gMFO1Coeff[] = { fis_gMFO1Coeff1, fis_gMFO1Coeff2 };
FIS_TYPE** fis_gMFOCoeff[] = { fis_gMFO0Coeff, fis_gMFO1Coeff };

// Input membership function set
int fis_gMFI0[] = { 0, 0, 0, 0 };
int* fis_gMFI[] = { fis_gMFI0 };

// Output membership function set
int fis_gMFO0[] = { 0, 0 };
int fis_gMFO1[] = { 0, 0 };
int* fis_gMFO[] = { fis_gMFO0, fis_gMFO1 };

// Rule Weights
FIS_TYPE fis_gRWeight[] = { 1, 1, 1, 1 };

// Rule Type
int fis_gRType[] = { 1, 1, 1, 1 };

// Rule Inputs
int fis_gRI0[] = { 1 };
int fis_gRI1[] = { 2 };
int fis_gRI2[] = { 3 };
int fis_gRI3[] = { 4 };
int* fis_gRI[] = { fis_gRI0, fis_gRI1, fis_gRI2, fis_gRI3 };

// Rule Outputs
int fis_gRO0[] = { 1, 2 };
int fis_gRO1[] = { 1, 2 };
int fis_gRO2[] = { 2, 2 };
int fis_gRO3[] = { 2, 1 };
int* fis_gRO[] = { fis_gRO0, fis_gRO1, fis_gRO2, fis_gRO3 };

// Input range Min
FIS_TYPE fis_gIMin[] = { 0 };

// Input range Max
FIS_TYPE fis_gIMax[] = { 40 };

// Output range Min
FIS_TYPE fis_gOMin[] = { 0, 0 };

// Output range Max
FIS_TYPE fis_gOMax[] = { 1.3, 1.3 };

//*************************
// Data dependent support functions for Fuzzy Inference System
//*************************
FIS_TYPE fis_MF_out(FIS_TYPE** fuzzyRuleSet, FIS_TYPE x, int o) {
  FIS_TYPE mfOut;
  int r;

  for (r = 0; r < fis_gcR; ++r) {
    int index = fis_gRO[r][o];
    if (index > 0) {
      index = index - 1;
      mfOut = (fis_gMF[fis_gMFO[o][index]])(x, fis_gMFOCoeff[o][index]);
    } else if (index < 0) {
      index = -index - 1;
      mfOut = 1 - (fis_gMF[fis_gMFO[o][index]])(x, fis_gMFOCoeff[o][index]);
    } else {
      mfOut = 0;
    }

    fuzzyRuleSet[0][r] = fis_min(mfOut, fuzzyRuleSet[1][r]);
  }
  return fis_array_operation(fuzzyRuleSet[0], fis_gcR, fis_max);
}

FIS_TYPE fis_defuzz_centroid(FIS_TYPE** fuzzyRuleSet, int o) {
  FIS_TYPE step = (fis_gOMax[o] - fis_gOMin[o]) / (FIS_RESOLUSION - 1);
  FIS_TYPE area = 0;
  FIS_TYPE momentum = 0;
  FIS_TYPE dist, slice;
  int i;

  // calculate the area under the curve formed by the MF outputs
  for (i = 0; i < FIS_RESOLUSION; ++i) {
    dist = fis_gOMin[o] + (step * i);
    slice = step * fis_MF_out(fuzzyRuleSet, dist, o);
    area += slice;
    momentum += slice * dist;
  }

  return ((area == 0) ? ((fis_gOMax[o] + fis_gOMin[o]) / 2) : (momentum / area));
}

//*************************
// Fuzzy Inference System
//*************************
void fis_evaluate() {
  FIS_TYPE fuzzyInput0[] = { 0, 0, 0, 0 };
  FIS_TYPE* fuzzyInput[fis_gcI] = {
    fuzzyInput0,
  };
  FIS_TYPE fuzzyOutput0[] = { 0, 0 };
  FIS_TYPE fuzzyOutput1[] = { 0, 0 };
  FIS_TYPE* fuzzyOutput[fis_gcO] = {
    fuzzyOutput0,
    fuzzyOutput1,
  };
  FIS_TYPE fuzzyRules[fis_gcR] = { 0 };
  FIS_TYPE fuzzyFires[fis_gcR] = { 0 };
  FIS_TYPE* fuzzyRuleSet[] = { fuzzyRules, fuzzyFires };
  FIS_TYPE sW = 0;

  // Transforming input to fuzzy Input
  int i, j, r, o;
  for (i = 0; i < fis_gcI; ++i) {
    for (j = 0; j < fis_gIMFCount[i]; ++j) {
      fuzzyInput[i][j] =
        (fis_gMF[fis_gMFI[i][j]])(g_fisInput[i], fis_gMFICoeff[i][j]);
    }
  }

  int index = 0;
  for (r = 0; r < fis_gcR; ++r) {
    if (fis_gRType[r] == 1) {
      fuzzyFires[r] = FIS_MAX;
      for (i = 0; i < fis_gcI; ++i) {
        index = fis_gRI[r][i];
        if (index > 0)
          fuzzyFires[r] = fis_min(fuzzyFires[r], fuzzyInput[i][index - 1]);
        else if (index < 0)
          fuzzyFires[r] = fis_min(fuzzyFires[r], 1 - fuzzyInput[i][-index - 1]);
        else
          fuzzyFires[r] = fis_min(fuzzyFires[r], 1);
      }
    } else {
      fuzzyFires[r] = FIS_MIN;
      for (i = 0; i < fis_gcI; ++i) {
        index = fis_gRI[r][i];
        if (index > 0)
          fuzzyFires[r] = fis_max(fuzzyFires[r], fuzzyInput[i][index - 1]);
        else if (index < 0)
          fuzzyFires[r] = fis_max(fuzzyFires[r], 1 - fuzzyInput[i][-index - 1]);
        else
          fuzzyFires[r] = fis_max(fuzzyFires[r], 0);
      }
    }

    fuzzyFires[r] = fis_gRWeight[r] * fuzzyFires[r];
    sW += fuzzyFires[r];
  }

  if (sW == 0) {
    for (o = 0; o < fis_gcO; ++o) {
      g_fisOutput[o] = ((fis_gOMax[o] + fis_gOMin[o]) / 2);
    }
  } else {
    for (o = 0; o < fis_gcO; ++o) {
      g_fisOutput[o] = fis_defuzz_centroid(fuzzyRuleSet, o);
    }
  }
}
