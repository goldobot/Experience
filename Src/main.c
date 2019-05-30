/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2019 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

#include "../PCA9685/pca9685.h"
#include "../SX1509/sx1509.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define JRO     // used to disable code during tests, MAKE SURE TO COMMENT OUT !!!
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;
DMA_HandleTypeDef hdma_i2c1_tx;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim6;
TIM_HandleTypeDef htim16;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */

// Variables liées au controle des moteurs PaP
extern uint8_t sm_busy;
extern uint16_t sm1_steps;  // steps à effectuer pour le mouvement en cours
extern uint16_t sm2_steps;
uint16_t sm1_pos = 0;   // position courante
uint16_t sm2_pos = 0;

// Table des positions des servos de la figurine (sera envoyée par DMA)
extern uint16_t servos[16]; // 8 servos, pour chacun, Ton et Toff

// Mot d'état des rubans de LED du décors, envoyé au SX1509 par I²C après chaque transfer DMA au PCA9685
uint8_t LEDs = 0;   // Eteintes par défaut

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_I2C1_Init(void);
static void MX_TIM16_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM6_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

// ====== Pilotage des PaP par interruption de TIM16 et TIM2 ========

// arguments : coordonnées (en mm) et vitesse (en mm/s) entre 3 et 100
void move (uint16_t Y, uint16_t Z, uint16_t S)
{
    // Tests de sécurité :
    if ((Y < 0 ) || (Y > 300)) return;  // portée horizontale
    if ((Z < 0 ) || (Z > 100)) return;  // portée verticale
    if ((S < 3 ) || (S > 100)) return;  // vitesse
    if (SW1 == 0) return;               // interrupteur de mise en sécurité / "homing"

    // Attendre la fin du mouvement précédent
    while (sm_busy > 0);
    // On active le busy
    sm_busy = 2;    // 2 parce qu'il y a deux moteurs
    // Nettoyage de signal : évite un temps trop court entre la dernière impulsion du mouvement précédent et la première impulsion du nouveau mouvement
    HAL_Delay (1);
    // Calcul du nombre de pas à effectuer, et de leur direction :
    // Axe 1 :
    // Conversion de la position demandée en pas moteur :
    uint16_t sm1_pos2 = Y * 25;
    // Déterminaison du sens de déplacement et de la distance à franchir :
    if (sm1_pos2 > sm1_pos)
        {
            // sens aller
            sm1_steps = (sm1_pos2 - sm1_pos) << 1;  // nombre de pas à effectuer, x2 car l'ISR du timer doit s'exécuter deux fois par pas
            // controle du signal de direction :
            HAL_GPIO_WritePin(SM1_DIR_GPIO_Port, SM1_DIR_Pin, GPIO_PIN_SET);
        }
    else
        {
            // sens retour
            sm1_steps = (sm1_pos - sm1_pos2) << 1;
            // controle du signal de direction :
            HAL_GPIO_WritePin(SM1_DIR_GPIO_Port, SM1_DIR_Pin, GPIO_PIN_RESET);
        }
    // Axe 2 :
    // Conversion de la position demandée en pas moteur :
    uint16_t sm2_pos2 = Z * 25;
    // Déterminaison du sens de déplacement et de la distance à franchir :
    if (sm2_pos2 > sm2_pos)
        {
            // sens aller
            sm2_steps = (sm2_pos2 - sm2_pos) << 1;  // nombre de pas à effectuer, x2 car l'ISR du timer doit s'exécuter deux fois par pas
            // controle du signal de direction :
            HAL_GPIO_WritePin(SM2_DIR_GPIO_Port, SM2_DIR_Pin, GPIO_PIN_SET);
        }
    else
        {
            // sens retour
            sm2_steps = (sm2_pos - sm2_pos2) << 1;
            // controle du signal de direction :
            HAL_GPIO_WritePin(SM2_DIR_GPIO_Port, SM2_DIR_Pin, GPIO_PIN_RESET);
        }
    // Calcul des fréquences timers
    // On commence par calculer la distance totale à parcourir :
    volatile double dist = sqrt (((double) sm1_steps * (double) sm1_steps) + ((double) sm2_steps * (double) sm2_steps));
    // Conversion de la vitesse de mm/s en pas/s :
    volatile double spd = S * 50;   // 25 x 2 : 25 pas par tour + doublé car l'interruption du timer aura lieu deux fois par pas
    // La figurine doit couvrir la distance "dist", en pas, à une vitesse de "spd", en pas par seconde
    // Calcul de la durée du mouvement coordonné :
    volatile double duration = dist / spd;    // resultat en secondes
    // Calcul de la période pour le timer de SM1 :
    // SM1 doit effectuer "sm1_steps" pas en "duration" secondes, donc la vitesse en pas/s sur cet axe est sm1_steps/duration : ce ratio est le nombre de pas à effectuer par seconde
    volatile double sm1_spd = sm1_steps / duration;
    // Le timer tourne à 1.44 MHz : pour produire la fréquence voulue, la période en cycles sera de 1.44 M / sm1_spd
    volatile double sm1_period = 1440000.0 / sm1_spd; // cette valeur devrait être inférieure à 65535 si la vitesse de mouvement est d'au moins 3 mm/s
    // SM2 : meme calculs :
    volatile double sm2_spd = sm2_steps / duration;
    volatile double sm2_period = 1440000.0 / sm2_spd; // cette valeur devrait être inférieure à 65535 si la vitesse de mouvement est d'au moins 3 mm/s

#ifdef JRO // raccourcissement de la trajectoire réelle afin d'aider à la visualisation sur scope
    sm1_steps /= 25;
    sm2_steps /= 25;
#endif

    // Paramétrage des timers
    htim16.Instance->ARR = (uint16_t) sm1_period;
    htim16.Instance->CCR1 = ((uint16_t) sm1_period) >> 1;   // on centre l'interruption sur la période
    htim2.Instance->ARR = (uint16_t) sm2_period;
    htim2.Instance->CCR1 = ((uint16_t) sm2_period) >> 1;   // on centre l'interruption sur la période
    // Lancement des timers
    HAL_TIM_OC_Start_IT(&htim2, 0);
    HAL_TIM_OC_Start_IT(&htim16, 0);
    // Mise à jour de la position courante aux nouvelles coordonnées
    sm1_pos = sm1_pos2;
    sm2_pos = sm2_pos2;
}

// Pilotage des servos de la figurine, fonction à interface simplifiée
// Argument : un tableau de huit valeurs 8 bits pour les 7 servos et la PWM des LED des yeux
// Cette fonction s'exécute en un peu moins de 1 ms, et elle est bloquante.
void pose (uint16_t* positions)
{
    servos[1] = positions[0];
    servos[3] = positions[1];
    servos[5] = positions[2];
    servos[7] = positions[3];
    servos[9] = positions[4];
    servos[11] = positions[5];
    servos[13] = positions[6];
    servos[15] = positions[7];
#ifndef JRO
    pca_writebuff16b (&hi2c1, PCA9685_LED8_ON_L, servos, 32);   // Remplacer PCA9685_LED8_ON_L par le premier canal utilisé sur la figurine, si nécessaire
#endif
}


/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */
  

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_USART2_UART_Init();
  MX_I2C1_Init();
  MX_TIM16_Init();
  MX_TIM3_Init();
  MX_TIM2_Init();
  MX_TIM6_Init();
  /* USER CODE BEGIN 2 */
  HAL_TIM_Base_Start_IT(&htim6); // Start_IT necessaire pour activer l'interruption de ce timer.

  // Init des périphériques I²C
#ifndef JRO
  sxinit(&hi2c1);
  pca_init(&hi2c1);
#endif

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */


  // Init des positions initiales des servos
#ifndef JRO
  int k;
  for (k = 0; k < 16; k++)
      servos[k] = 0;
  pose (uint16_t[]{320, 320, 320, 320, 320, 320, 320, 0});  // Servos en position médiane, LED éteintes
#endif

  // Activation des servos "electron" et "toit"
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);     // Electron
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);     // Toit
  // HAL_TIM_PWM_Start(&htim3, 2);     // Réserve

  // première boucle infinie, dédiée à la detection de robot et au lancement de l'électron
#ifndef JRO
  uint8_t presence_robot = 0;
#else
  uint8_t presence_robot = 10;  // bypass du test de présence robot, UNIQUEMENT EN DEVELOPPEMENT
#endif
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

    HAL_Delay (10);

    // On attend de détecter le robot, puis qu'il disparaisse de devant le capteur
    if (presence_robot < 10)
    {
        // tester le capteur ultrasons et mettre à jour presence_robot le cas échéant
        // on veut détecter le robot dix fois d'affilée pour être certain qu'on ne déclenche pas sur un malentendu.
        if (SENSOR == GPIO_PIN_SET)
            presence_robot++;
        else
            presence_robot = 0;
    }
    else
    {
        // on a détecté un robot durant dix cycles de 10 ms (histoire de filtrer tout bruit sur le capteur)
        // re-tester le capteur ultrasons : si le robot a quitté le champ du capteur, on lance l'électron et on quitte cette boucle infinie
#ifndef JRO
        if (SENSOR == GPIO_PIN_RESET)
        {
#endif
            // SEQUENCE SERVO :
            htim3.Instance->CCR1 = 256 + 0; // Mouvement à droite
            HAL_Delay (500);                // Pause 0.5 seconde
            htim3.Instance->CCR1 = 256 + 256; // Mouvement à gauche
            HAL_Delay (500);               // Pause 0.5 seconde
            htim3.Instance->CCR1 = 256 + 128;// Mouvement au centre
            break;
#ifndef JRO
        }
#endif
    }

  }

  // Deuxième boucle infinie, dédiée à la "choregraphie"
  while (1)
  {
   // Attitude de la figurine

   // Exemple de séquence de mouvement des PaP : chaque appel est bloquant, use wisely !
   //   move (50, 20, 3);
   //   move (0, 0, 100);
   // Shorter moves for debugging, at closer speeds
   move (2, 5, 10);     // "move" est non-bloquante, elle retourne des que le mouvement commence
   pose ((uint16_t[]){110, 530, 110, 530, 110, 530, 110, 2000});   // "pose" est bloquante et dure 1 ms. Les commandes de servo (ES9051) doivent être entre 110 et 530, la commande de LED peut aller de 0 à 4095
   move (0, 0, 15);
   pose ((uint16_t[]){530, 110, 530, 110, 530, 110, 530, 3000});

    //  HAL_Delay (10);
  }

  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_BYPASS;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_I2C1;
  PeriphClkInit.I2c1ClockSelection = RCC_I2C1CLKSOURCE_SYSCLK;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.Timing = 0x00702681;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Analogue filter 
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Digital filter 
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 50;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 1000;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_OC_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_TOGGLE;
  sConfigOC.Pulse = 20;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_OC_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */
  HAL_TIM_MspPostInit(&htim2);

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 280;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 5120;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 384;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */
  HAL_TIM_MspPostInit(&htim3);

}

/**
  * @brief TIM6 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM6_Init(void)
{

  /* USER CODE BEGIN TIM6_Init 0 */

  /* USER CODE END TIM6_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM6_Init 1 */

  /* USER CODE END TIM6_Init 1 */
  htim6.Instance = TIM6;
  htim6.Init.Prescaler = 50;
  htim6.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim6.Init.Period = 57600;
  htim6.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim6) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim6, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM6_Init 2 */

  /* USER CODE END TIM6_Init 2 */

}

/**
  * @brief TIM16 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM16_Init(void)
{

  /* USER CODE BEGIN TIM16_Init 0 */

  /* USER CODE END TIM16_Init 0 */

  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM16_Init 1 */

  /* USER CODE END TIM16_Init 1 */
  htim16.Instance = TIM16;
  htim16.Init.Prescaler = 50;
  htim16.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim16.Init.Period = 1000;
  htim16.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim16.Init.RepetitionCounter = 0;
  htim16.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim16) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_OC_Init(&htim16) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_TOGGLE;
  sConfigOC.Pulse = 20;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_OC_ConfigChannel(&htim16, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.BreakFilter = 0;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim16, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM16_Init 2 */

  /* USER CODE END TIM16_Init 2 */
  HAL_TIM_MspPostInit(&htim16);

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 38400;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/** 
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void) 
{
  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel2_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel2_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel2_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, SM2_DIR_Pin|BUCK_ENABLE_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, LED_Pin|SM1_DIR_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : SM2_DIR_Pin BUCK_ENABLE_Pin */
  GPIO_InitStruct.Pin = SM2_DIR_Pin|BUCK_ENABLE_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : SWITCH_1_Pin SWITCH_2_Pin */
  GPIO_InitStruct.Pin = SWITCH_1_Pin|SWITCH_2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : SENSOR_Pin SWITCH_H_Pin SWITCH_V_Pin */
  GPIO_InitStruct.Pin = SENSOR_Pin|SWITCH_H_Pin|SWITCH_V_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : LED_Pin SM1_DIR_Pin */
  GPIO_InitStruct.Pin = LED_Pin|SM1_DIR_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */

  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(char *file, uint32_t line)
{ 
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
