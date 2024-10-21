/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  ******************************************************************************
  * @attention
  *
  * Softare feito para trabalho de conclusão do curso técnico em eletrônica na
  * fundação liberato.
  *
  * @brief
  * O código cosiste em aquisicionar amostras do ADC, processar usando filtros
  * digitais IIR Chebyhev II na forma de biquads transpostos usando a
  * biblioteca DSP da CMSIS e por fim enviar o som filtrado de volta pelo DAC
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "string.h"
#include "stdio.h"
#include "math.h"
#include "arm_math.h"
#include "arm_const_structs.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define N_AMOSTRAS 128 						 // Amostras DMA buffer
#define BLOCK_SIZE 32						 // Tamanho de cada bloco de amostras
#define NUM_BLOCKS (N_AMOSTRAS / BLOCK_SIZE) // Numero de blocos de 32 amostras

#define MSG_SIZE 200 // Tamanho da mensagem da serial
#define Voffset 1165 // offset de 1.5V
#define IIR_ORDEM 40 // Ordem do filtro
#define IIR_SECOESBIQUAD IIR_ORDEM/2 // Numero de seções biquads
#define GANHO_AUDIO 5				 // Ganho na amplitude das amostras
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

DAC_HandleTypeDef hdac;
DMA_HandleTypeDef hdma_dac1;

TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;

UART_HandleTypeDef huart2;
DMA_HandleTypeDef hdma_usart2_tx;

/* USER CODE BEGIN PV */
int16_t medidas[2][N_AMOSTRAS];					// Valores do ADC em int16_t
uint16_t entra = 0;								// Linha do da matriz de amostras

float32_t entrada[N_AMOSTRAS];					// Valores não filtrados em float32
float32_t saida[N_AMOSTRAS];					// Valores filtrados em float32

float32_t *InputValuesf32_ptr = &entrada[0]; 	// declare Input pointer
float32_t *OutputValuesf32_ptr = &saida[0]; 	// declare Output pointer

int16_t amostras_de_saida[N_AMOSTRAS]; 			// Valores filtrados em int16
int16_t amostras_de_saida_py[N_AMOSTRAS];

uint8_t soltou = 0; // Variável para debounce
uint8_t estado = 0; // Variável para debounce

float32_t estado_biquad_coracao[IIR_ORDEM] = {0};
float32_t estado_biquad_pulmao[IIR_ORDEM] = {0};

float32_t coefs_biquad_coracao_negados[5*IIR_SECOESBIQUAD] = {
		0.00008961896804606912195646090113, 0.00002950709423280477720669297270, 0.00008961896804606910840393374507, 1.41151657231764571243104455788853, -0.50008321107119668713636428947211,
		1.00000000000000000000000000000000, -1.45272403227823065918755673919804, 1.00000000000000000000000000000000, 1.48721817981854065315872048813617, -0.56701045829571883327702153110295,
		1.00000000000000000000000000000000, -1.77462507126759705400331768032629, 0.99999999999999977795539507496869, 1.59377661128072922736009786603972, -0.66124185926229284149258091929369,
		1.00000000000000000000000000000000, -1.87386911961864788267462245130446, 1.00000000000000000000000000000000, 1.69284173992479969328428524022456, -0.74891929786118838219266535816132,
		1.00000000000000000000000000000000, -1.91590653990214776491995962715009, 0.99999999999999988897769753748435, 1.77125564502976340364170937391464, -0.81847039650147002110003313646303,
		1.00000000000000000000000000000000, -1.93712729209319012291246053791838, 1.00000000000000022204460492503131, 1.83000370938214751426187376637245, -0.87083113674767054224190587774501,
		1.00000000000000000000000000000000, -1.94894287076975136763223872549133, 1.00000000000000000000000000000000, 1.87388832064673049870862087118439, -0.91031580908123721496139069131459,
		1.00000000000000000000000000000000, -1.95578867538759193500652600050671, 0.99999999999999988897769753748435, 1.90753933409056153358562823996181, -0.94109662628393442318497363885399,
		1.00000000000000000000000000000000, -1.95962574095055086509375996683957, 1.00000000000000044408920985006262, 1.93458364200577337399522548366804, -0.96648143994030888404012102910201,
		1.00000000000000000000000000000000, -1.99999903425160541203808861610014, 0.99999999999999977795539507496869, 1.98694228711753995142430539999623, -0.98698626056918903337589199509239,
		1.00000000000000000000000000000000, -1.99999150577920059568270971794846, 1.00000000000000000000000000000000, 1.98743469100059355625376156240236, -0.98748590375360500992485413007671,
		1.00000000000000000000000000000000, -1.99997745061261578136679872841341, 1.00000000000000044408920985006262, 1.98835046963899730876335070206551, -0.98841505176057042891812898233184,
		1.00000000000000000000000000000000, -1.96136143971428511534327299159486, 1.00000000000000022204460492503131, 1.95779429640365343345820292597637, -0.98906306359198425948164867804735,
		1.00000000000000000000000000000000, -1.99995864881066864882086520083249, 1.00000000000000000000000000000000, 1.98958115513641686789014784153551, -0.98966336070438953864680797778419,
		1.00000000000000000000000000000000, -1.99993730509238143433492496114923, 0.99999999999999988897769753748435, 1.99101549426325252767355777905323, -0.99111740227632305622051944737905,
		1.00000000000000000000000000000000, -1.99991569038331329366542377101723, 0.99999999999999966693309261245304, 1.99256338282221578950270668428857, -0.99268497680061307253396307714866,
		1.00000000000000000000000000000000, -1.99989586849850953242935247544665, 0.99999999999999988897769753748435, 1.99416352492602300117141567170620, -0.99430298833067043418054709036369,
		1.00000000000000000000000000000000, -1.99987953650627403590078756678849, 1.00000000000000000000000000000000, 1.99578084122502286135159010882489, -0.99593493621585427177933524944820,
		1.00000000000000000000000000000000, -1.99986796039529823865166235918878, 0.99999999999999988897769753748435, 1.99740002394738880298064032103866, -0.99756447793350766506392801602487,
		1.00000000000000000000000000000000, -1.99986196871683374887140871578595, 1.00000000000000000000000000000000, 1.99901878719740699885676349367714, -0.99918865653074695476476563271717
};


float32_t coefs_biquad_pulmao_negados[5*IIR_SECOESBIQUAD] = {
		0.00042795945535743291259292431228, 0.00078419610124759648629721997537, 0.00042795945535743280417270706373, 0.36006952496092770044100461745984, -0.04017843778983194724663263741604,
		1.00000000000000000000000000000000, 0.89252630667361554372973841964267, 1.00000000000000022204460492503131, 0.45364598330667088577072831867554, -0.11426379035281511442612156770338,
		1.00000000000000000000000000000000, -0.00789340334742653568222969795443, 1.00000000000000022204460492503131, 0.60928054330644876301903423154727, -0.23685418792503715179620371600322,
		1.00000000000000000000000000000000, -0.58572416952888761709772325048107, 1.00000000000000022204460492503131, 0.78655854161661087342594100846327, -0.37583541856267932423918409767793,
		1.00000000000000000000000000000000, -0.93020186479068056595309599288157, 1.00000000000000000000000000000000, 0.95603586280143426634481329529081, -0.50874172366101455544651344098384,
		1.00000000000000000000000000000000, -1.13801800197083524679442234628368, 1.00000000000000044408920985006262, 1.10368686534339244253999368083896, -0.62582862518073678614882737747394,
		1.00000000000000000000000000000000, -1.26604034701301815246665682934690, 1.00000000000000000000000000000000, 1.22632645818834040696287956961896, -0.72598550552075735708967840764672,
		1.00000000000000000000000000000000, -1.34481150438902763433191012154566, 0.99999999999999977795539507496869, 1.32614379114315106278354505775496, -0.81229960168106829510037414365797,
		1.00000000000000000000000000000000, -1.39056301542469706866711476322962, 0.99999999999999988897769753748435, 1.40724697489610450773511729494203, -0.88947333091777203151195863028988,
		1.00000000000000000000000000000000, -1.99995367099179355285798465047264, 1.00000000000000000000000000000000, 1.91012500769369930608831964491401, -0.91220935560076643611182589665987,
		1.00000000000000000000000000000000, -1.99959441012323813069428979360964, 1.00000000000000000000000000000000, 1.91401082757505913711781886377139, -0.91641869915284701608726436461438,
		1.00000000000000000000000000000000, -1.99893245251865669942503700440284, 0.99999999999999988897769753748435, 1.92091447759981837073439692176180, -0.92391012215008927466186605670373,
		1.00000000000000000000000000000000, -1.99806396650676076909292078198632, 1.00000000000000000000000000000000, 1.92965388699269579930728468752932, -0.93340847415936045727846703812247,
		1.00000000000000000000000000000000, -1.99710028175798015581676736474037, 0.99999999999999988897769753748435, 1.93924426026518226251482701627538, -0.94383068733215291779004019190324,
		1.00000000000000000000000000000000, -1.99614686940687402483263213071041, 1.00000000000000000000000000000000, 1.94907041188968976896944695909042, -0.95447510993830875225540921746870,
		1.00000000000000000000000000000000, -1.41165524464389213754600405081874, 1.00000000000000022204460492503131, 1.47407397048799415628650422149803, -0.96286507962753364875396755451220,
		1.00000000000000000000000000000000, -1.99529117178488846207073947880417, 0.99999999999999988897769753748435, 1.95883936173909445344065716199111, -0.96497998633401582679880448267795,
		1.00000000000000000000000000000000, -1.99459875844276868228632793034194, 1.00000000000000022204460492503131, 1.96847696383129378006060505867936, -0.97522002983841216128269024920883,
		1.00000000000000000000000000000000, -1.99411456477494386518856117618270, 1.00000000000000022204460492503131, 1.97804028273214482780417711182963, -0.98521654709997008581012778449804,
		1.00000000000000000000000000000000, -1.99386602788905920036199859168846, 1.00000000000000000000000000000000, 1.98765837629074870740453206963139, -0.99507565999924563193701487762155
};

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_ADC1_Init(void);
static void MX_TIM3_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_DAC_Init(void);
static void MX_TIM4_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
// Instâncias de cada filtro
arm_biquad_cascade_df2T_instance_f32 SC;
arm_biquad_cascade_df2T_instance_f32 SP;
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */
	// inicializando os filtros
	arm_biquad_cascade_df2T_init_f32(&SC, IIR_SECOESBIQUAD, &coefs_biquad_coracao_negados[0], &estado_biquad_coracao[0]);
	arm_biquad_cascade_df2T_init_f32(&SP, IIR_SECOESBIQUAD, &coefs_biquad_pulmao_negados[0], &estado_biquad_pulmao[0]);
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
  MX_ADC1_Init();
  MX_TIM3_Init();
  MX_USART2_UART_Init();
  MX_DAC_Init();
  MX_TIM4_Init();
  /* USER CODE BEGIN 2 */
  HAL_TIM_OC_Start(&htim3, TIM_CHANNEL_1);
  HAL_TIM_OC_Start(&htim4, TIM_CHANNEL_1);

  //Começa a enviar as medidas para a memoria no caso 16
  HAL_ADC_Start_DMA(&hadc1, &medidas[entra][0], N_AMOSTRAS);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
   while (1)
  {
    /* USER CODE END WHILE */
	   if(HAL_GPIO_ReadPin(B1_GPIO_Port, B1_Pin) == 1 && soltou == 0){
	   		soltou = 1;
	   		estado = ~estado;
	   	}

	   	if(HAL_GPIO_ReadPin(B1_GPIO_Port, B1_Pin) == 0){
	   	   soltou = 0;
	   	}
    /* USER CODE BEGIN 3 */
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

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_RISING;
  hadc1.Init.ExternalTrigConv = ADC_EXTERNALTRIGCONV_T3_TRGO;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DMAContinuousRequests = ENABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_15CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief DAC Initialization Function
  * @param None
  * @retval None
  */
static void MX_DAC_Init(void)
{

  /* USER CODE BEGIN DAC_Init 0 */

  /* USER CODE END DAC_Init 0 */

  DAC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN DAC_Init 1 */

  /* USER CODE END DAC_Init 1 */

  /** DAC Initialization
  */
  hdac.Instance = DAC;
  if (HAL_DAC_Init(&hdac) != HAL_OK)
  {
    Error_Handler();
  }

  /** DAC channel OUT1 config
  */
  sConfig.DAC_Trigger = DAC_TRIGGER_T4_TRGO;
  sConfig.DAC_OutputBuffer = DAC_OUTPUTBUFFER_ENABLE;
  if (HAL_DAC_ConfigChannel(&hdac, &sConfig, DAC_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN DAC_Init 2 */

  /* USER CODE END DAC_Init 2 */

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
  htim3.Init.Prescaler = 104;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 99;
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
  if (HAL_TIM_OC_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_TOGGLE;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_OC_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */
  HAL_TIM_MspPostInit(&htim3);

}

/**
  * @brief TIM4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM4_Init(void)
{

  /* USER CODE BEGIN TIM4_Init 0 */

  /* USER CODE END TIM4_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 104;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 99;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim4, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_OC_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_TOGGLE;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_OC_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */
  HAL_TIM_MspPostInit(&htim4);

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
  huart2.Init.BaudRate = 230400;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
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
  __HAL_RCC_DMA2_CLK_ENABLE();
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Stream5_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream5_IRQn);
  /* DMA1_Stream6_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream6_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream6_IRQn);
  /* DMA2_Stream0_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream0_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_0|SAIDA_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : PC0 SAIDA_Pin */
  GPIO_InitStruct.Pin = GPIO_PIN_0|SAIDA_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : ENTRADA_Pin */
  GPIO_InitStruct.Pin = ENTRADA_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(ENTRADA_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LD2_Pin */
  GPIO_InitStruct.Pin = LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

int clamp_value(int16_t value ) {
	if(estado == 0) value = (value*3) + 1400;
	else value = (value*1.4) + 1400;
    if (value < 0.00) return 0;
    if (value > 4095) return 4095;
    return value;
}

/**
  * Processamento da metade dos dados aquisicionados pelo ADC
  * Checagem do botão | 0 para pulmão | 1 para coração
  */
void HAL_ADC_ConvHalfCpltCallback(ADC_HandleTypeDef* hadc){

	for(int amostra = 0; amostra < (N_AMOSTRAS/2); amostra++)
	{
		entrada[amostra] = (float32_t)medidas[entra][amostra]-Voffset;

	}

	// LOOP DE FILTRAGEM DOS SONS
	for (int k = 0; k < (NUM_BLOCKS/2); k++)
	{
		if(estado == 0){
			arm_biquad_cascade_df2T_f32(&SP, InputValuesf32_ptr + (k*BLOCK_SIZE), OutputValuesf32_ptr + (k*BLOCK_SIZE), BLOCK_SIZE);
		}
		else
			arm_biquad_cascade_df2T_f32(&SC, InputValuesf32_ptr + (k*BLOCK_SIZE), OutputValuesf32_ptr + (k*BLOCK_SIZE), BLOCK_SIZE);

	}

	for (int k = 0; k < (N_AMOSTRAS/2); k++)
	{
		int16_t var = clamp_value((int16_t)saida[k]);
		amostras_de_saida_py[k] = ((int16_t)saida[k])*GANHO_AUDIO; 	// Passando de float para int | para ser enviado pela serial
		amostras_de_saida[k] = var;		// Passando de float para int | para ser enviado pelo DAC
	}
}

/**
  * Processamento da segunda metade dos dados aquisicionados pelo ADC
  * Início do próximo ADC e transmissão de dados
  */
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc){
	int16_t ent = entra; // Var para manter o processamento das amostras

	if(++entra>1) entra=0; // Começo do próximo aquisionamento
	HAL_ADC_Start_DMA(&hadc1, &medidas[entra][0], N_AMOSTRAS);
	HAL_DAC_Start_DMA(&hdac, DAC_CHANNEL_1, &amostras_de_saida[0], N_AMOSTRAS, DAC_ALIGN_12B_R);

	//HAL_GPIO_WritePin(GPIOC, GPIO_PIN_0, SET);
	for(int amostra = (N_AMOSTRAS/2); amostra < N_AMOSTRAS; amostra++)
	{
		entrada[amostra] = (float32_t)medidas[ent][amostra]-Voffset;
	}

	for (int k = (NUM_BLOCKS/2); k < NUM_BLOCKS; k++)
	{
		if(estado == 0){
			arm_biquad_cascade_df2T_f32(&SP, InputValuesf32_ptr + (k*BLOCK_SIZE), OutputValuesf32_ptr + (k*BLOCK_SIZE), BLOCK_SIZE);
		}
		else
			arm_biquad_cascade_df2T_f32(&SC, InputValuesf32_ptr + (k*BLOCK_SIZE), OutputValuesf32_ptr + (k*BLOCK_SIZE), BLOCK_SIZE);
	}


	for (int k = (N_AMOSTRAS/2); k < N_AMOSTRAS; k++)
	{
		int16_t var = clamp_value((int16_t)saida[k]);
		amostras_de_saida_py[k] = ((int16_t)saida[k])*GANHO_AUDIO;	// Passando de float para int | para ser enviado pela serial
		amostras_de_saida[k] = var;		// Passando de float para int | para ser enviado pelo DAC
	}
	HAL_UART_Transmit_DMA(&huart2, &amostras_de_saida_py[0], N_AMOSTRAS*2);
	//HAL_GPIO_WritePin(GPIOC, GPIO_PIN_0, RESET);
}


/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
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
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
