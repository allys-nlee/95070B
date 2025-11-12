#include "vex.h"
#include "auton.h"
#include <iostream>
#include <string>

using namespace vex;
using namespace std;

// Drivetrain PID
void drivePID(double targetdegrees, double drivekp = 0.5, double driveki = 0.25, double drivekd = 0.3) {
   Inertial.setRotation(0, degrees);
   double error = targetdegrees;
   double integral = 0;
   double lasterror = error;
   double lspeed;
   double rspeed;
   double prevdegrees = FL.position(degrees);
   double startrotation = Inertial.rotation(degrees);
   double rotdif = 0;
   double krotdif = 0;
   double count = 0;
   //printf("target: %f degrees\n", targetdegrees);
   FL.setPosition(0, degrees);
   BL.setPosition(0, degrees);
   FR.setPosition(0, degrees);
   BR.setPosition(0, degrees);
   ML.setPosition(0, degrees);
   MR.setPosition(0, degrees);

while (fabs(error) > 3) {
       double measureddegrees = (FL.position(degrees) + FR.position(degrees)) / 2;
       error = targetdegrees - measureddegrees;
       if(fabs(measureddegrees - prevdegrees) < 2){
           count++; //add to count
       } else { //if not being stalled
           count = 0;
       }

       if (count > 40) { //exit when stuck for ~500 ms
           FL.stop(brake);
           FR.stop(brake);
           ML.stop(brake);
           MR.stop(brake);
           BR.stop(brake);
           BL.stop(brake);
           //printf("exit1\n");
           return;
       }

       prevdegrees = measureddegrees;

    //Integral windup
       if ((fabs(error) < fabs(targetdegrees * 0.3)) && (fabs(integral) < 150)) {
           integral += error;
       }

       if(fabs(error) < 3) {
         FL.stop(brake);
         FR.stop(brake);
         ML.stop(brake);
         MR.stop(brake);
         BL.stop(brake);
         BR.stop(brake);
         //printf("exit2\n");
         double measureddegrees = (FL.position(degrees) + FR.position(degrees)) / 2;
         error = targetdegrees - measureddegrees;
         printf("final: %f degrees, error %f\n", measureddegrees, error);
         return;
       }

        // PID calculation
        double pid = error * drivekp + integral * driveki + (error - lasterror) * drivekd;
        //printf("FL: %f degrees, error %f %f %f %f\n", FL.position(deg), error, error * drivekp, integral * driveki, (error - lasterror) * drivekd);
        lspeed = pid;
        rspeed = pid;

        // // Heading correction
        // rotdif = Inertial.rotation(degrees) - startrotation;
        // krotdif = rotdif * 4;

        // Apply correction
        double lfinal = lspeed; //- krotdif;
        double rfinal = rspeed; //+ krotdif;

        // Set motor directions based on target sign
        directionType dir = (targetdegrees >= 0) ? vex::forward : vex::reverse;
        FL.spin(dir, fabs(lfinal), rpm);
        ML.spin(dir, fabs(lfinal), rpm);
        BL.spin(dir, fabs(lfinal), rpm);
        FR.spin(dir, fabs(rfinal), rpm);
        MR.spin(dir, fabs(rfinal), rpm);
        BR.spin(dir, fabs(rfinal), rpm);

        lasterror = error;
        wait(20, msec);
    }
    //printf("whileexit\n");
    double measureddegrees = (FL.position(degrees) + FR.position(degrees)) / 2;
    error = targetdegrees - measureddegrees;
    printf("final: %f degrees, error %f\n", measureddegrees, error);
    FL.stop(brake);
    FR.stop(brake);
    ML.stop(brake);
    MR.stop(brake);
    BR.stop(brake);
    BL.stop(brake);
}


void skillsdrivePID(double targetdegrees, double drivekp = 0.67, double driveki = 0.0017, double drivekd = 0.55) {
   Inertial.setRotation(0, degrees);
   double error = targetdegrees;
   double integral = 0;
   double lasterror = error;
   double lspeed;
   double rspeed;
   double prevdegrees = FL.position(degrees);
   double startrotation = Inertial.rotation(degrees);
   double rotdif = 0;
   double krotdif = 0;
   double count = 0;
//    printf("target: %f degrees\n", targetdegrees);
   FL.setPosition(0, degrees);
   BL.setPosition(0, degrees);
   FR.setPosition(0, degrees);
   BR.setPosition(0, degrees);
   ML.setPosition(0, degrees);
   MR.setPosition(0, degrees);

while (fabs(error) > 0.5) {
       double measureddegrees = (FL.position(degrees) + FR.position(degrees)) / 2;
       error = targetdegrees - measureddegrees;
       if(fabs(measureddegrees - prevdegrees) < 2){
           count++; //add to count
       } else { //if not being stalled
           count = 0;
       }

       if (count > 40) { //exit when stuck for 400 ms
           FL.stop(brake);
           FR.stop(brake);
           ML.stop(brake);
           MR.stop(brake);
           BR.stop(brake);
           BL.stop(brake);
        //    printf("exit1\n");
           return;
       }

       prevdegrees = measureddegrees;

    //Integral windup
       if ((fabs((error) < targetdegrees / (10*3)) && (fabs(integral) < 300))) {
           integral += error;
       }

       if(fabs(error) < 2) {
         FL.stop(brake);
         FR.stop(brake);
         ML.stop(brake);
         MR.stop(brake);
         BL.stop(brake);
         BR.stop(brake);
        //  printf("exit2\n");
         return;
       }

        // PID calculation
        double pid = (error * drivekp + integral * driveki + (error - lasterror) * drivekd) * 0.2;
        // printf("FL: %f degrees, error %f %f %f %f\n", FL.position(deg), error, error * drivekp, integral * driveki, (error - lasterror) * drivekd);
        lspeed = pid;
        rspeed = pid;

        // // Heading correction
        // rotdif = Inertial.rotation(degrees) - startrotation;
        // krotdif = rotdif * 4;

        // Apply correction
        double lfinal = lspeed; //- krotdif;
        double rfinal = rspeed; //+ krotdif;

        // Set motor directions based on target sign
        directionType dir = (targetdegrees >= 0) ? vex::forward : vex::reverse;
        FL.spin(dir, fabs(lfinal), rpm);
        ML.spin(dir, fabs(lfinal), rpm);
        BL.spin(dir, fabs(lfinal), rpm);
        FR.spin(dir, fabs(rfinal), rpm);
        MR.spin(dir, fabs(rfinal), rpm);
        BR.spin(dir, fabs(rfinal), rpm);

        lasterror = error;
        wait(20, msec);
    }
    // printf("whileexit\n");
    double measureddegrees = (FL.position(degrees) + FR.position(degrees)) / 2;
    error = targetdegrees - measureddegrees;
    //printf("final: %f degrees, error %f\n", measureddegrees, error);
    FL.stop(brake);
    FR.stop(brake);
    ML.stop(brake);
    MR.stop(brake);
    BR.stop(brake);
    BL.stop(brake);
}


void thatonethingsheldonwants(double targetdegrees, double drivekp = 0.67, double driveki = 0.0017, double drivekd = 0.55) {
   Inertial.setRotation(0, degrees);
   double error = targetdegrees;
   double integral = 0;
   double lasterror = error;
   double lspeed;
   double rspeed;
   double prevdegrees = FL.position(degrees);
   double startrotation = Inertial.rotation(degrees);
   double rotdif = 0;
   double krotdif = 0;
   double count = 0;
//    printf("target: %f degrees\n", targetdegrees);
   FL.setPosition(0, degrees);
   BL.setPosition(0, degrees);
   FR.setPosition(0, degrees);
   BR.setPosition(0, degrees);
   ML.setPosition(0, degrees);
   MR.setPosition(0, degrees);

while (fabs(error) > 0.5) {
       double measureddegrees = (FL.position(degrees) + FR.position(degrees)) / 2;
       error = targetdegrees - measureddegrees;
       if(fabs(measureddegrees - prevdegrees) < 2){
           count++; //add to count
       } else { //if not being stalled
           count = 0;
       }

       if (count > 40) { //exit when stuck for 400 ms
           FL.stop(brake);
           FR.stop(brake);
           ML.stop(brake);
           MR.stop(brake);
           BR.stop(brake);
           BL.stop(brake);
        //    printf("exit1\n");
           return;
       }

       prevdegrees = measureddegrees;

    //Integral windup
       if ((fabs((error) < targetdegrees / (10*3)) && (fabs(integral) < 300))) {
           integral += error;
       }

       if(fabs(error) < 2) {
         FL.stop(brake);
         FR.stop(brake);
         ML.stop(brake);
         MR.stop(brake);
         BL.stop(brake);
         BR.stop(brake);
        //  printf("exit2\n");
         return;
       }

        // PID calculation
        double pid = (error * drivekp + integral * driveki + (error - lasterror) * drivekd) * 0.4;
        // printf("FL: %f degrees, error %f %f %f %f\n", FL.position(deg), error, error * drivekp, integral * driveki, (error - lasterror) * drivekd);
        lspeed = pid;
        rspeed = pid;

        // // Heading correction
        // rotdif = Inertial.rotation(degrees) - startrotation;
        // krotdif = rotdif * 4;

        // Apply correction
        double lfinal = lspeed; //- krotdif;
        double rfinal = rspeed; //+ krotdif;

        // Set motor directions based on target sign
        directionType dir = (targetdegrees >= 0) ? vex::forward : vex::reverse;
        FL.spin(dir, fabs(lfinal), rpm);
        ML.spin(dir, fabs(lfinal), rpm);
        BL.spin(dir, fabs(lfinal), rpm);
        FR.spin(dir, fabs(rfinal), rpm);
        MR.spin(dir, fabs(rfinal), rpm);
        BR.spin(dir, fabs(rfinal), rpm);

        lasterror = error;
        wait(20, msec);
    }
    // printf("whileexit\n");
    double measureddegrees = (FL.position(degrees) + FR.position(degrees)) / 2;
    error = targetdegrees - measureddegrees;
    //printf("final: %f degrees, error %f\n", measureddegrees, error);
    FL.stop(brake);
    FR.stop(brake);
    ML.stop(brake);
    MR.stop(brake);
    BR.stop(brake);
    BL.stop(brake);
}


void slowdrivePID(double targetdegrees, double drivekp = 0.67, double driveki = 0.0017, double drivekd = 0.55) {
   Inertial.setRotation(0, degrees);
   double error = targetdegrees;
   double integral = 0;
   double lasterror = error;
   double lspeed;
   double rspeed;
   double prevdegrees = FL.position(degrees);
   double startrotation = Inertial.rotation(degrees);
   double rotdif = 0;
   double krotdif = 0;
   double count = 0;
//    printf("target: %f degrees\n", targetdegrees);
   FL.setPosition(0, degrees);
   BL.setPosition(0, degrees);
   FR.setPosition(0, degrees);
   BR.setPosition(0, degrees);
   ML.setPosition(0, degrees);
   MR.setPosition(0, degrees);

while (fabs(error) > 0.5) {
       double measureddegrees = (FL.position(degrees) + FR.position(degrees)) / 2;
       error = targetdegrees - measureddegrees;
       if(fabs(measureddegrees - prevdegrees) < 2){
           count++; //add to count
       } else { //if not being stalled
           count = 0;
       }

       if (count > 40) { //exit when stuck for 400 ms
           FL.stop(brake);
           FR.stop(brake);
           ML.stop(brake);
           MR.stop(brake);
           BR.stop(brake);
           BL.stop(brake);
        //    printf("exit1\n");
           return;
       }

       prevdegrees = measureddegrees;

    //Integral windup
       if ((fabs((error) < targetdegrees / (10*3)) && (fabs(integral) < 300))) {
           integral += error;
       }

       if(fabs(error) < 2) {
         FL.stop(brake);
         FR.stop(brake);
         ML.stop(brake);
         MR.stop(brake);
         BL.stop(brake);
         BR.stop(brake);
        //  printf("exit2\n");
         return;
       }

        // PID calculation
        double pid = (error * drivekp + integral * driveki + (error - lasterror) * drivekd) * 0.75;
        // printf("FL: %f degrees, error %f %f %f %f\n", FL.position(deg), error, error * drivekp, integral * driveki, (error - lasterror) * drivekd);
        lspeed = pid;
        rspeed = pid;

        // // Heading correction
        // rotdif = Inertial.rotation(degrees) - startrotation;
        // krotdif = rotdif * 4;

        // Apply correction
        double lfinal = lspeed; //- krotdif;
        double rfinal = rspeed; //+ krotdif;

        // Set motor directions based on target sign
        directionType dir = (targetdegrees >= 0) ? vex::forward : vex::reverse;
        FL.spin(dir, fabs(lfinal), rpm);
        ML.spin(dir, fabs(lfinal), rpm);
        BL.spin(dir, fabs(lfinal), rpm);
        FR.spin(dir, fabs(rfinal), rpm);
        MR.spin(dir, fabs(rfinal), rpm);
        BR.spin(dir, fabs(rfinal), rpm);

        lasterror = error;
        wait(20, msec);
    }
    // printf("whileexit\n");
    double measureddegrees = (FL.position(degrees) + FR.position(degrees)) / 2;
    error = targetdegrees - measureddegrees;
    //printf("final: %f degrees, error %f\n", measureddegrees, error);
    FL.stop(brake);
    FR.stop(brake);
    ML.stop(brake);
    MR.stop(brake);
    BR.stop(brake);
    BL.stop(brake);
}


void slowturnPID(double turndegrees, double turnkp = 2.8, double turnki = 0.035, double turnkd = 5.0) {
    Inertial.setRotation(0, degrees);  // reset to 0
    double error = turndegrees;
    double integral = 0;
    double lasterror = error;
    double speed = 0;
    //printf("target: %f degrees\n", turndegrees);
    double currentdeg = Inertial.rotation(degrees);
    double prevdeg = currentdeg;
    double count = 0;

    while (fabs(error) > 2) {
        currentdeg = Inertial.rotation(degrees);
        double actualTurned = currentdeg - prevdeg;
        error = turndegrees - actualTurned;

        if(fabs(error) < 2) {
            FL.stop(brake);
            FR.stop(brake);
            ML.stop(brake);
            MR.stop(brake);
            BL.stop(brake);
            BR.stop(brake);
            currentdeg = Inertial.rotation(degrees);
            printf("deg %f\n", currentdeg);
            break;
        }

        //printf("deg: %f degrees, error %f %f %f %f, speed %f\n", currentdeg, error, error * turnkp, integral * turnki, (error - lasterror) * turnkd, speed);
        if (fabs(currentdeg - prevdeg) < 0.3) {
            count++;
        } else {
            count = 0;
        }

        if (count > 20) { //exit after 400 msec
            //printf("stuck\n");
            break;
        }

     //Integral windup
       if ((fabs(error) < fabs(turndegrees * 0.9)) && (fabs(integral) < 1000)) {
           integral += error;
       }

        speed = (error * turnkp + integral * turnki + (error - lasterror) * turnkd) * 0.6;
        //printf("deg: %f degrees, error %f %f %f %f %f\n", currentdeg, error, error * turnkp, integral * turnki, (error - lasterror) * turnkd, speed);

        if (speed > 600) speed = 600;
        if (speed < -600) speed = -600;

        FL.spin(fwd, speed, rpm);
        ML.spin(fwd, speed, rpm);
        BL.spin(fwd, speed, rpm);
        FR.spin(fwd, -speed, rpm);
        MR.spin(fwd, -speed, rpm);
        BR.spin(fwd, -speed, rpm);

        lasterror = error;
        wait(20, msec);
    }
    //printf("whileexit\n");
    FL.stop(brake);
    ML.stop(brake);
    BL.stop(brake);
    FR.stop(brake);
    MR.stop(brake);
    BR.stop(brake);
    wait(200, msec);
    currentdeg = Inertial.rotation(degrees);
    printf("deg %f\n", currentdeg);
}


void turnPID(double turndegrees, double turnkp = 2.8, double turnki = 0.035, double turnkd = 5.0) {
    Inertial.setRotation(0, degrees);  // reset to 0
    double error = turndegrees;
    double integral = 0;
    double lasterror = error;
    double speed = 0;
    //printf("target: %f degrees\n", turndegrees);
    double currentdeg = Inertial.rotation(degrees);
    double prevdeg = currentdeg;
    double count = 0;

    while (fabs(error) > 2) {
        currentdeg = Inertial.rotation(degrees);
        double actualTurned = currentdeg - prevdeg;
        error = turndegrees - actualTurned;

        if(fabs(error) < 2) {
            FL.stop(brake);
            FR.stop(brake);
            ML.stop(brake);
            MR.stop(brake);
            BL.stop(brake);
            BR.stop(brake);
            currentdeg = Inertial.rotation(degrees);
            printf("deg %f\n", currentdeg);
            break;
        }

        //printf("deg: %f degrees, error %f %f %f %f, speed %f\n", currentdeg, error, error * turnkp, integral * turnki, (error - lasterror) * turnkd, speed);
        if (fabs(currentdeg - prevdeg) < 0.3) {
            count++;
        } else {
            count = 0;
        }

        if (count > 20) { //exit after 400 msec
            //printf("stuck\n");
            break;
        }

     //Integral windup
       if ((fabs(error) < fabs(turndegrees * 0.9)) && (fabs(integral) < 1000)) {
           integral += error;
       }

        speed = error * turnkp + integral * turnki + (error - lasterror) * turnkd;
        //printf("deg: %f degrees, error %f %f %f %f %f\n", currentdeg, error, error * turnkp, integral * turnki, (error - lasterror) * turnkd, speed);

        if (speed > 600) speed = 600;
        if (speed < -600) speed = -600;

        FL.spin(fwd, speed, rpm);
        ML.spin(fwd, speed, rpm);
        BL.spin(fwd, speed, rpm);
        FR.spin(fwd, -speed, rpm);
        MR.spin(fwd, -speed, rpm);
        BR.spin(fwd, -speed, rpm);

        lasterror = error;
        wait(20, msec);
    }
    //printf("whileexit\n");
    FL.stop(brake);
    ML.stop(brake);
    BL.stop(brake);
    FR.stop(brake);
    MR.stop(brake);
    BR.stop(brake);
    wait(200, msec);
    currentdeg = Inertial.rotation(degrees);
    printf("deg %f\n", currentdeg);
}


//inchestodegrees code for drivePID
double inchestodegrees(double inches){
   return ((inches / (4.0 * M_PI)) * 360.0) * (7.0/4.0);
}

//random auton intake controls
void intaking() {
     intake.spin(vex::forward, 11000, voltageUnits::mV);
     outake.spin(vex::reverse, 1500, voltageUnits::mV);
}

void outtake() {
     intake.spin(vex::reverse, 8000, voltageUnits::mV);
     outake.spin(vex::reverse, 8000, voltageUnits::mV);
}

void directscoring() {
     intake.spin(vex::forward, 11000, voltageUnits::mV);
     outake.spin(vex::forward, 11000, voltageUnits::mV);
}

void middlegoal() {
     outake.spin(vex::forward, 8500, voltageUnits::mV);
}

void intakeStop() {
     intake.stop(brake);
     outake.stop(brake);
}

void hardstop() {
     FL.stop(brake);
     FR.stop(brake);
     ML.stop(brake);
     MR.stop(brake);
     BR.stop(brake);
     BL.stop(brake);
}

void tsfpmo() {
    FL.spin(vex::reverse, 9000, voltageUnits::mV);
    ML.spin(vex::reverse, 9000, voltageUnits::mV);
    BL.spin(vex::reverse, 9000, voltageUnits::mV);
    FR.spin(vex::reverse, 9000, voltageUnits::mV);
    MR.spin(vex::reverse, 9000, voltageUnits::mV);
    BR.spin(vex::reverse, 9000, voltageUnits::mV);
    wait(500, msec);
    hardstop();
}

void sheldonisapoo() {
     intake.spin(vex::forward, 10000, voltageUnits::mV);
     outake.spin(vex::reverse, 1000, voltageUnits::mV);
     wait(1100, msec);
     intake.stop(brake);
     outake.stop(brake);
}

void sheldonisaskillspoo() {
    FL.spin(vex::forward, 6000, voltageUnits::mV);
    ML.spin(vex::forward, 6000, voltageUnits::mV);
    BL.spin(vex::forward, 6000, voltageUnits::mV);
    FR.spin(vex::forward, 6000, voltageUnits::mV);
    MR.spin(vex::forward, 6000, voltageUnits::mV);
    BR.spin(vex::forward, 6000, voltageUnits::mV);
    wait(1300, msec);
    hardstop();
}

void sheldonisverytall() {
    FL.spin(vex::reverse, 3000, voltageUnits::mV);
    ML.spin(vex::reverse, 3000, voltageUnits::mV);
    BL.spin(vex::reverse, 3000, voltageUnits::mV);
    FR.spin(vex::reverse, 3000, voltageUnits::mV);
    MR.spin(vex::reverse, 3000, voltageUnits::mV);
    BR.spin(vex::reverse, 3000, voltageUnits::mV);
    wait(200, msec);
    FL.spin(vex::forward, 3000, voltageUnits::mV);
    ML.spin(vex::forward, 3000, voltageUnits::mV);
    BL.spin(vex::forward, 3000, voltageUnits::mV);
    FR.spin(vex::forward, 3000, voltageUnits::mV);
    MR.spin(vex::forward, 3000, voltageUnits::mV);
    BR.spin(vex::forward, 3000, voltageUnits::mV);
    wait(700, msec);
    hardstop();
}

void fakeantijam() {
    outtake();
    wait(300, msec);
    directscoring();
}

void auton1(){
    intaking();
    thatonethingsheldonwants(inchestodegrees(28));
    turnPID(105);
    drivePID(inchestodegrees(28.5));
    intakeStop();
    turnPID(55.0);
    mlm.set(true);
    thatonethingsheldonwants(inchestodegrees(19.0));
    sheldonisapoo();
    slowdrivePID(inchestodegrees(-28));
    directscoring();
    wait(1500, msec);
    fakeantijam();
    wait(2000, msec);
    intakeStop();
    // drivePID(inchestodegrees(8));
    // tsfpmo();
}
void auton2(){
    intaking();
    thatonethingsheldonwants(inchestodegrees(26));
    turnPID(-105);
    drivePID(inchestodegrees(29.0));
    intakeStop();
    turnPID(-57.0);
    mlm.set(true);
    thatonethingsheldonwants(inchestodegrees(19.0));
    sheldonisapoo();
    slowdrivePID(inchestodegrees(-28));
    directscoring();
    wait(1500, msec);
    fakeantijam();
    wait(2000, msec);
    intakeStop();
}
void auton3(){
    intake.spin(vex::forward, 11000, voltageUnits::mV);
    thatonethingsheldonwants(inchestodegrees(29.0));
    intake.stop(brake);
    turnPID(-117);
    drivePID(inchestodegrees(-12));
    outake.spin(vex::forward, 8000, voltageUnits::mV);
    wait(350, msec);
    outake.stop(brake);
    slowdrivePID(inchestodegrees(48.5));
    intakeStop();
    turnPID(-47.0);
    mlm.set(true);
    wait(100, msec);
    drivePID(inchestodegrees(11.5));
    hardstop();
    intaking();
    wait(1100, msec);
    slowdrivePID(inchestodegrees(-28));
    directscoring();
    wait(1500, msec);
    fakeantijam();
    wait(2000, msec);
    intakeStop();
}

// void auton3(){
//     //go to first loader
//     skillsdrivePID(inchestodegrees(31.5)); //31.5
//     turnPID(88.5);
//     mlm.set(true);
//     wait(500, msec);

//     //get blocks
//     intaking();
//     drivePID(inchestodegrees(8.67));
//     wait(500, msec);
//     sheldonisverytall();
//     wait(2000, msec);
//     intakeStop();

//     //score on first goal
//     thatonethingsheldonwants(inchestodegrees(-30));
//     directscoring();
//     mlm.set(false);
//     wait(2000, msec);
//     fakeantijam();
//     wait(1000, msec);
//     intakeStop();
    
//     //travel to other loader
//     drivePID(inchestodegrees(13));
//     turnPID(88); //90 or 89
//     wait(500, msec);
//     skillsdrivePID(inchestodegrees(99.0));
//     wait(500, msec);
//     turnPID(-89);

//     //align to goal
//     slowdrivePID(inchestodegrees(-18));
//     directscoring();
//     wait(2000, msec);
//     intakeStop();

//     //get blocks
//     mlm.set(true);
//     intaking();
//     skillsdrivePID(inchestodegrees(34));
//     wait(1500, msec);
//     sheldonisverytall();
//     wait(2500, msec);
//     intakeStop();
//     turnPID(4.67);

//     //score on second goal
//     skillsdrivePID(inchestodegrees(-32));
//     drivePID(inchestodegrees(-7));
//     directscoring();
//     mlm.set(false);
//     wait(2000, msec);
//     fakeantijam();
//     wait(3000, msec);
//     intakeStop();

//     //park.
//     drivePID(inchestodegrees(14));
//     turnPID(-60);
//     drivePID(inchestodegrees(33.0));
//     turnPID(-26);
//     outtake();
//     skillsdrivePID(inchestodegrees(38.6));
//     //sheldonisaskillspoo();
//     }