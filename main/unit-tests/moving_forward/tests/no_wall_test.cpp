#include "../MoveForward.h"
#include <assert.h>
#include <math.h>
#include <stdio.h>

int main() {
  const NoWallSettings settings = {
      4314.0f/7.0f, 4321.5f/7.0f, 30, {1, 0.10f, 0, 40}
  };
  NoWallController controller;
  float encoderError, encoderPwm;

  auto commands=controller.update(1000,1000,70,settings,0.05f,
      encoderError,encoderPwm);
  assert(commands.left==70 && commands.right==70); // Entry captures both references.

  commands=controller.update(5314,5322,70,settings,0.05f,
      encoderError,encoderPwm);
  assert(fabsf(encoderError)<0.6f); // Within one integer tick of equal travel.

  commands=controller.update(5334,5322,70,settings,0.05f,
      encoderError,encoderPwm);
  assert(encoderError>0 && encoderPwm>0 && commands.left<70 && commands.right==70);
  const float initialPwm=encoderPwm;
  for(int i=0;i<20;++i) {
    commands=controller.update(5334,5322,70,settings,0.05f,
        encoderError,encoderPwm);
  }
  assert(encoderPwm>initialPwm); // Ki corrects a persistent travel mismatch.

  controller.reset();
  controller.update(0,0,70,settings,0.05f,encoderError,encoderPwm);
  commands=controller.update(0,20,70,settings,0.05f,encoderError,encoderPwm);
  assert(encoderError<0 && encoderPwm<0 && commands.right<70 && commands.left==70);

  assert(fabsf(settings.encoderPid.ki-0.10f)<0.0001f);

  // Saturation must not accumulate enough integral to keep steering afterward.
  controller.reset();
  controller.update(0,0,35,settings,0.05f,encoderError,encoderPwm);
  for(int i=0;i<1000;++i)
    controller.update(100,0,35,settings,0.05f,encoderError,encoderPwm);
  controller.update(616,617,70,settings,0.05f,encoderError,encoderPwm);
  assert(fabsf(encoderPwm)<0.6f);

  assert(noWallApproachSpeed(35,70,140)==70);
  assert(noWallApproachSpeed(140,70,140)==140);
  assert(noWallApproachSpeed(35,70,50)==50);
  assert(noWallApproachSpeed(0,70,140)==0);
  assert(noWallYawCorrection(5,0,2,0.15f,15)==-10);
  assert(noWallYawCorrection(-20,0,2,0.15f,15)==15);
  // Positive encoder PWM slows left; negative MPU trim also slows left.
  commands=noWallCombinedCommands(140,60,10,-8,0.8f);
  assert(commands.left==132 && commands.right==140);
  // Opposing encoder and MPU corrections cancel into one steering command.
  commands=noWallCombinedCommands(140,60,10,8,0.8f);
  assert(commands.left==140 && commands.right==136);
  commands=noWallCombinedCommands(70,60,-40,15,0.8f);
  assert(commands.left==70 && commands.right==60); // Combined output respects floor.
  commands=noWallCombinedCommands(140,60,10,15,0.0f);
  assert(commands.left==130 && commands.right==140); // MPU failure restores 100% encoder.
  const NoWallSettings rolling={4314.0f/7.0f,4321.5f/7.0f,60,{1.7f,0.10f,0,40}};
  controller.reset();
  controller.update(0,0,70,rolling,0.046f,encoderError,encoderPwm);
  commands=controller.update(4223,4097,70,rolling,0.046f,encoderError,encoderPwm);
  assert(commands.left==60 && commands.right==70);
  // Same-named motor/encoder plant: reducing PWM reduces that wheel's travel.
  // A pre-existing mismatch must shrink in both directions (negative feedback).
  for (int ahead=0; ahead<2; ++ahead) {
    controller.reset();
    controller.update(0,0,140,rolling,0.03f,encoderError,encoderPwm);
    float leftPos=ahead==0?60.0f:0.0f;
    float rightPos=ahead==1?60.0f:0.0f;
    for (int step=0;step<200;++step) {
      commands=controller.update((unsigned long)lroundf(leftPos),
          (unsigned long)lroundf(rightPos),140,rolling,0.03f,encoderError,encoderPwm);
      leftPos+=commands.left*0.03f*rolling.leftTicksPerCell/620.0f;
      rightPos+=commands.right*0.03f*rolling.rightTicksPerCell/620.0f;
    }
    assert(fabsf(encoderError)<5.0f);
  }
  puts("PASS: case 3 encoder/MPU steering, fallback, limits and rolling floors");
}
