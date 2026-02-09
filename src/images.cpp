#include "vex.h"
#include "images.h"

// Helper: draw a line with italic shear applied
static void drawItalicLine(int baseX, int baseY, int x1, int y1, int x2, int y2, float shear, int charH) {
  int dx1 = baseX + x1 + (int)(shear * (1.0f - (float)y1 / charH));
  int dy1 = baseY + y1;
  int dx2 = baseX + x2 + (int)(shear * (1.0f - (float)y2 / charH));
  int dy2 = baseY + y2;
  Brain.Screen.drawLine(dx1, dy1, dx2, dy2);
}

void drawTeamNumber() {
  const int charW = 50;
  const int charH = 90;
  const int spacing = 15;
  const int penW = 7;
  const float shear = 12.0f;

  const int totalW = 5 * charW + 4 * spacing;
  const int baseX = (480 - totalW) / 2;
  const int baseY = (240 - charH) / 2;

  Brain.Screen.setPenWidth(penW);
  Brain.Screen.setPenColor(white);

  int cx = baseX;

  // '9'
  drawItalicLine(cx, baseY, 0, 0, charW, 0, shear, charH);
  drawItalicLine(cx, baseY, 0, 0, 0, charH / 2, shear, charH);
  drawItalicLine(cx, baseY, 0, charH / 2, charW, charH / 2, shear, charH);
  drawItalicLine(cx, baseY, charW, 0, charW, charH, shear, charH);
  drawItalicLine(cx, baseY, 0, charH, charW, charH, shear, charH);
  cx += charW + spacing;

  // '1'
  drawItalicLine(cx, baseY, charW / 2, 0, charW / 2, charH, shear, charH);
  drawItalicLine(cx, baseY, charW / 4, charH / 6, charW / 2, 0, shear, charH);
  drawItalicLine(cx, baseY, charW / 4, charH, 3 * charW / 4, charH, shear, charH);
  cx += charW + spacing;

  // '8'
  drawItalicLine(cx, baseY, 0, 0, charW, 0, shear, charH);
  drawItalicLine(cx, baseY, 0, charH / 2, charW, charH / 2, shear, charH);
  drawItalicLine(cx, baseY, 0, charH, charW, charH, shear, charH);
  drawItalicLine(cx, baseY, 0, 0, 0, charH, shear, charH);
  drawItalicLine(cx, baseY, charW, 0, charW, charH, shear, charH);
  cx += charW + spacing;

  // '1'
  drawItalicLine(cx, baseY, charW / 2, 0, charW / 2, charH, shear, charH);
  drawItalicLine(cx, baseY, charW / 4, charH / 6, charW / 2, 0, shear, charH);
  drawItalicLine(cx, baseY, charW / 4, charH, 3 * charW / 4, charH, shear, charH);
  cx += charW + spacing;

  // 'F'
  drawItalicLine(cx, baseY, 0, 0, charW, 0, shear, charH);
  drawItalicLine(cx, baseY, 0, charH / 2, 3 * charW / 4, charH / 2, shear, charH);
  drawItalicLine(cx, baseY, 0, 0, 0, charH, shear, charH);
}
