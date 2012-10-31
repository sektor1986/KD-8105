#ifndef __BACKLIGHT_H
#define __BACKLIGHT_H

void InitBacklight(void);
void SetValueBacklight(void);
void SetBacklightNull(void);
__interrupt void Backlight_IRQ(void);

#endif
