#include "../NoWallControl.h"
#include <assert.h>
#include <math.h>
#include <stdio.h>

int main() {
  const NoWallSettings settings = {
      4359.0f/7.0f, 4334.5f/7.0f, 30, {1, 0.10f, 0, 40}
  };
  NoWallController controller;
  float encoderError, encoderPwm;

  auto commands=controller.update(1000,1000,70,settings,0.05f,
      encoderError,encoderPwm);
  assert(commands.left==70 && commands.right==70); // Entry captures both references.

  commands=controller.update(5359,5334,70,settings,0.05f,
      encoderError,encoderPwm);
  assert(fabsf(encoderError)<0.6f); // Within one integer tick of equal travel.

  commands=controller.update(5379,5334,70,settings,0.05f,
      encoderError,encoderPwm);
  assert(encoderError>0 && encoderPwm>0 && commands.right<70 && commands.left==70);
  const float initialPwm=encoderPwm;
  for(int i=0;i<20;++i) {
    commands=controller.update(5379,5334,70,settings,0.05f,
        encoderError,encoderPwm);
  }
  assert(encoderPwm>initialPwm); // Ki corrects a persistent travel mismatch.

  controller.reset();
  controller.update(0,0,70,settings,0.05f,encoderError,encoderPwm);
  commands=controller.update(0,20,70,settings,0.05f,encoderError,encoderPwm);
  assert(encoderError<0 && encoderPwm<0 && commands.left<70 && commands.right==70);

  assert(fabsf(settings.encoderPid.ki-0.10f)<0.0001f);

  // Saturation must not accumulate enough integral to keep steering afterward.
  controller.reset();
  controller.update(0,0,35,settings,0.05f,encoderError,encoderPwm);
  for(int i=0;i<1000;++i)
    controller.update(100,0,35,settings,0.05f,encoderError,encoderPwm);
  controller.update(623,619,70,settings,0.05f,encoderError,encoderPwm);
  assert(fabsf(encoderPwm)<0.6f);

  puts("PASS: encoder-only case 3 uses manual calibration, Ki correction, and anti-windup");
}
