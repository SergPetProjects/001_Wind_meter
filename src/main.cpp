// Программа для работы с анеморумбометром М63М-1

#include <Arduino.h>
#include <Wire.h>              //  Подключаем библиотеку для работы с шиной I2C
#include <LiquidCrystal_I2C.h> //  Подключаем библиотеку для работы с LCD дисплеем по шине I2C

#define _MEASUREMENT_PERIOD 10000   // Период измерения скорости ветра(10 с)
#define _MEASUREMENT_AZIMUTH 120000 // Период вычисления азимута флюгарки(2 мин)
#define _ARRAY_PERIOD_VOLUME 60     // Размер массива для хранения скорости ветра (60 = 10 мин / 10000 мс)
#define _ARRAY_FAZA_VOLUME 600      // Размер массива для хранения азимута
#define _STEP 5                     // Разбиение по азимутам с шагом 5 градусов

LiquidCrystal_I2C lcd(0x27, 20, 4); //  Объявляем  объект библиотеки, указывая параметры дисплея (адрес I2C = 0x27, количество столбцов = 20, количество строк = 4)

void isr();
void period();
void faza();
void bubbleSort();
void lcdFirst();
void lcdPrint();
void azimuthSort();
void azimuth();

volatile bool period_flag = true; // разрешение на измерение периода
volatile bool faza_flag = false;  // разрешение на измерение фазы
// volatile bool sort_flag = true;    // разрешение на сортировку массива
volatile uint32_t tmr_period = 0, tmr_old = 0, tmr_faza, tmr_faza_old;
volatile uint32_t period_time, period_middle, period_max;
volatile uint16_t counter = 0;
uint32_t period_array[_ARRAY_PERIOD_VOLUME], sort_array[_ARRAY_PERIOD_VOLUME];
uint32_t azimuth_array[_ARRAY_FAZA_VOLUME], azimuth_faza[_ARRAY_FAZA_VOLUME];
uint32_t tmr2 = 0, tmr10 = 0;
uint16_t azimuth_selection[360 / _STEP]; // 360 градусов разбито по 5 градусов
uint16_t i_period = 0, i_faza = 0, i_faza_max = 0, azimuth_middle = 0;

void setup()
{
  Serial.begin(115200);
  lcdFirst();
  attachInterrupt(0, isr, FALLING);  // прерывание по LOW на D2
  attachInterrupt(1, faza, FALLING); // прерывание по LOW на D3
  pinMode(2, INPUT_PULLUP);          // назначить выводу порт ввода с подтяжкой
  pinMode(3, INPUT_PULLUP);          // назначить выводу порт ввода с подтяжкой
}

void loop()
{
  if (millis() - tmr10 >= _MEASUREMENT_PERIOD)
  {
    period(); // Расчёт периода вращения вертушки
    tmr10 = millis();
    // period_flag = true; // разрешает запись времени прихода импульса от D2

    period_array[i_period] = period_time; // заполнение массива данными о периоде (затирается самое старое)
    if (++i_period >= _ARRAY_PERIOD_VOLUME)
      i_period = 0;

    // if (sort_flag)
    memcpy(sort_array, period_array, _ARRAY_PERIOD_VOLUME * sizeof(period_array[0])); // копирует массив period_array в sort_array
    bubbleSort();
    period_middle = sort_array[_ARRAY_PERIOD_VOLUME / 2];
    period_max = sort_array[_ARRAY_PERIOD_VOLUME - 3];

    lcdPrint(); // вывод на LCD
  }

  if (millis() - tmr2 >= _MEASUREMENT_AZIMUTH)
  {
    i_faza_max = i_faza;
    memcpy(azimuth_array, azimuth_faza, i_faza_max * sizeof(azimuth_faza[0])); // копирует актуальную часть массива azimuth_faza в azimuth_array
    i_faza = 0;
    azimuthSort();
    tmr2 = millis();
  }
}

// Счёт импульсов от D2 для вычисления периода вращения вертушки
void isr()
{
  if (period_flag)
  {
    tmr_period = micros();
    period_flag = false;
    faza_flag = true; // Включается ожидание импульса от D3 для вычисления фазы (азимута)
    counter++;        // импульсы считаем тут
    Serial.print("Количество импульсов за 10 сек. = ");
    Serial.println(counter);
  }
}

// Расчёт периода вращения вертушки
void period()
{
  uint32_t tmr1 = micros();
  if (counter == 0)
  {
    counter = 1;
  }
  period_time = (tmr1 - tmr_old) / counter; // Расчёт периода вращения вертушки
  counter = 0;
  tmr_old = tmr1;
  Serial.print("Период вращения вертушки (мс) = ");
  Serial.println(period_time / 1000.0);
  Serial.println();
}

// Измерение фазы положения флюгарки (азимута)
// Работае по прерыванию на D3 (LOW) на основе предварительной фиксации времени импульса от D2
void faza()
{
  if (faza_flag)
  {
    tmr_faza_old = tmr_faza;
    tmr_faza = micros();
    azimuth(); // Расчёт азимута флюгарки
    period_flag = true;
    faza_flag = false;
  }
}

// Расчёт азимута флюгарки
void azimuth()
{
  azimuth_faza[i_faza] = (359 * (tmr_faza - tmr_period)) / (tmr_faza - tmr_faza_old); // азимут в градусах
  Serial.print("Азимут флюгарки = ");
  Serial.println(azimuth_faza[i_faza]);
  Serial.println();
  if (azimuth_faza[i_faza] <= 359)
    i_faza++; // для затирания неверного значения азимута на следующей итерации
}

// Выборка из массива фаз самого частого значения азимута с дискретностью 5 градусов
void azimuthSort()
{
  int max = 0;
  for (int i = 0; i < 72; i++)
    azimuth_selection[i] = 0; // обнуление массива

  for (int i = 0; i < i_faza_max; i++)
  {
    uint32_t select = azimuth_array[i] / _STEP;
    if (max < ++azimuth_selection[select])
    {
      max = azimuth_selection[select]; // поиска max в массиве
      azimuth_middle = select * _STEP;
      Serial.print("azimuth_middle = ");
      Serial.println(azimuth_middle);
    }
    Serial.print(select * _STEP);
    Serial.print(" - ");
    Serial.println(azimuth_selection[select]);
    }
}

// Сортировка массива (пузырьковая)
void bubbleSort()
{
  // sort_flag = false;
  for (int j = 0; j + 1 < _ARRAY_PERIOD_VOLUME; ++j)
  {
    for (int i = 0; i + 1 < _ARRAY_PERIOD_VOLUME - j; ++i)
    {
      if (sort_array[i] > sort_array[i + 1])
      {
        unsigned long temp = sort_array[i];
        sort_array[i] = sort_array[i + 1];
        sort_array[i + 1] = temp;
      }
    }
  }

  // sort_flag = true;
}

// Первоначальный вывод на экран LCD
void lcdFirst()
{
  lcd.init(); // initialize the lcd
  lcd.backlight();
  lcd.setCursor(7, 0); //  Устанавливаем курсор в позицию (столбец, строка)
  lcd.print("METEO");
  lcd.setCursor(4, 3); //  Устанавливаем курсор в позицию (столбец, строка)
  lcd.print("Serg Grebnev");
  delay(1000);
  lcd.clear();
  lcd.setCursor(0, 0); //  Устанавливаем курсор в позицию (столбец, строка)
  lcd.print("BETEP:");
  lcd.setCursor(0, 1); //  Устанавливаем курсор в позицию (столбец, строка)
  lcd.print("  ==>");
  lcd.setCursor(0, 2); //  Устанавливаем курсор в позицию (столбец, строка)
  lcd.print("A3NMYT:");
}

// Вывод информации на экран LCD
void lcdPrint()
{
  lcd.setCursor(8, 0); //  Устанавливаем курсор в позицию (столбец, строка)
  lcd.print(period_middle);
  lcd.print("    ");
  lcd.setCursor(8, 1); //  Устанавливаем курсор в позицию (столбец, строка)
  lcd.print(period_max);
  lcd.print("  ");
  lcd.print(5 / 3);
  lcd.setCursor(8, 2); //  Устанавливаем курсор в позицию (столбец, строка)
  lcd.print(azimuth_middle);
  lcd.print("    ");
}