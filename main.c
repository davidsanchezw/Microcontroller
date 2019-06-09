/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  ** This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * COPYRIGHT(c) 2019 STMicroelectronics
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "..\..\Utils_SDBM.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
// Frecuencias de las Notas
	#define DO3 61156
	#define RE3 54794
	#define MI3 48780
	#define FA3 45977
	#define SOL3 41025
	#define LA3 36363
	#define SI3 32520
	
	#define DO 30581
	#define RE 27239
	#define MI 24301
	#define FA 22910
	#define SOL 20408
	#define LA 18182
	#define SI 16198
	
	#define DO5 12140
	#define RE5 11461
	#define MI5 10825
	#define FA5 10217
	#define SOL5 9639
	#define LA5 9091
	#define SI5 8584
	
	#define DO3R 766
	#define RE3R 682
	#define MI3R 607
	#define FA3R 572
	#define SOL3R 511
	#define LA3R 453
	#define SI3R 405
	
	#define DOR 382
	#define RER 340
	#define MIR 304
	#define FAR 286
	#define SOLR 255
	#define LAR 227
	#define SIR 202
	
//Sostenidos
	#define DO_S 28860
	#define RE_S 25712
	#define FA_S 21622
	#define SOL_S 19263
	#define LA_S 17167 //Si Bemol
	
	#define PAUSA 0
	
	#define TIM4CCR1 40000
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc;

DAC_HandleTypeDef hdac;

LCD_HandleTypeDef hlcd;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */
int ccr; 									//ccr de la señal cuadrada
uint8_t changeMode = 1; 	//A uno para que ponga el título de Piano nada más iniciar
uint8_t changeMelodyTone; //Para cambiar la nota de la melodia
uint8_t changeDacTone; 		//Para cambiar la posicion en el seno
uint8_t texto[6] = "      ";//Seleccion de octava
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_LCD_Init(void);
static void MX_TS_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM4_Init(void);
static void MX_TIM3_Init(void);
static void MX_DAC_Init(void);
static void MX_ADC_Init(void);
static void MX_USART1_UART_Init(void);
/* USER CODE BEGIN PFP */
void activateTIM2(int newccr); //funcion para activar el timer de la señal cuadrada
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

void TIM2_IRQHandler(void){	//Interrupcion  del TIM2 (senal cuadrada)
	if((TIM2->SR & (1 << (1) )) != 0){
		TIM2->CCR1 += ccr;
		TIM2->SR = 0;
	}
}

void TIM3_IRQHandler(void){	//Interrupcion  del TIM3 (tiempo de cada nota en melodia)
	if((TIM3->SR & (1 << (1) )) != 0){
		changeMelodyTone = 1;
		TIM3->SR &= ~(0x0002); 
	}
}

void TIM4_IRQHandler(void){	//Interrupcion  del TIM4 (senal dac)
	if((TIM4->SR & (1 << (2) )) != 0){ //Canal 2
		changeDacTone = 1;
		TIM4->SR &= ~(0x0004); 
	}
}

void EXTI0_IRQHandler(void) { //Interrupcion  del boton (cambiar de modo)
	if (EXTI->PR!=0){
		if((GPIOC->IDR & (1 << (13))) != 0){
				changeMode = 1;
		}
		EXTI->PR = 0x01; //Se limpia e flag
	}
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
	
	unsigned char tecla_do[3] = {1,1,1};
  unsigned char tecla_re[3] = {1,1,1};
	unsigned char tecla_mi[3] = {1,1,1};
	unsigned char tecla_fa[3] = {1,1,1};
	unsigned char tecla_sol[3] = {1,1,1};
	unsigned char tecla_la[3] = {1,1,1};
	unsigned char tecla_si[3] = {1,1,1};
	uint8_t posSong = 0; //Usado para el modo piano y el modo melodia, siempre se resetea así que no hay problema 
	uint8_t nota_limite = 35; //Nota limite para la primerca cancion
	
	uint8_t posOnda = 0;
	uint8_t seno[20] = {0,6,24,53,88,128,167,202,231,249,255,249,231,202,167,128,88,53,24,6};

	uint16_t ccrForTimDAC = 0; 
	
  uint8_t modo = 1;

	uint16_t* notas;
	uint16_t* duraciones;
	
	uint8_t newsong;
		
		//MELODIAS
	uint16_t cancion1[]=
	{
		DO,DO,DO,FA,LA,
		DO,DO,DO,FA,LA,
		FA,FA,MI,MI,RE,RE,DO,PAUSA,
		
		DO,DO,DO,MI,SOL,
		DO,DO,DO,MI,SOL,
		DO,RE,DO,LA_S,LA,SOL,FA,PAUSA
	};
	uint16_t duraciones1[]=
	{
			250,250,250,250,500,
			250,250,250,250,500,
			200,200,200,200,200,200,600,150,
		
			250,250,250,250,500,
			250,250,250,250,500,
			200,200,200,200,200,200,600,1500
	};
	
	
	uint16_t cancion2[]=
		{
			FA, LA, SI, FA, LA, SI,
			FA, LA, SI, MI5, RE5, SI, 
			DO5, SI, SOL, MI, RE, MI, SOL, MI
		};
	uint16_t duraciones2[]=
	{
			250,250,500,250,250,500,
			250,250,250,250,500, 250,
			250,250,250,1250,250,250,250,1500
	};
	
	uint16_t cancion3[]=
		{
		LA,MI,FA,SOL,
		FA,MI,RE,
		RE,FA,LA,
		SOL,FA,MI,
		FA,SOL,LA,
		FA,RE,RE,PAUSA
		};
	uint16_t duraciones3[]=
	{
		250,250,200,350,
		200,200,350,
		200,200,350,
		200,200,350,
		200,400,400,
		350,400,400,500
	};

//Melodia predeterminada
	notas = cancion1;
	duraciones = duraciones1;

	
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
  MX_LCD_Init();
  MX_TS_Init();
  MX_TIM2_Init();
  MX_TIM4_Init();
  MX_TIM3_Init();
  MX_DAC_Init();
  MX_ADC_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */

	/* Inicialización del LCD */
	BSP_LCD_GLASS_Init();
	BSP_LCD_GLASS_BarLevelConfig(0);
	BSP_LCD_GLASS_Clear();
	
	
	/* Timer TIM2 -> senal rectangular */
	
		TIM2->CR1 = 0x0000;
		TIM2->CR2 = 0x0000;
		TIM2->SMCR = 0x0000;
		TIM2->DIER = 0x0002;
		
		TIM2->PSC = 1; //Preescalado de 1+1 = 2 -> 32.000.000Hz -> 16.000.000Hz
		TIM2->CNT = 0;
		TIM2->ARR = 0xFFFF;
		//No del todo necesario, pero para tener un valor inicializado
		ccr = DO;
		TIM2->CCR1 = ccr;
		//Salida HW: Togle y
		TIM2->CCMR1 |= (1 << 4); 
		TIM2->CCMR1 |= (1 << 5);
		TIM2->CCMR1 &= ~(1 << 6);	// CCyS = 0 (TOC)
		//No Activamos todavía el CCyE para habilitar salida
		TIM2->CCER = 0x0001;
		
		//Actualización de registros del contador y limpiamos el registro de estado
		TIM2->EGR |= 0x0001; 
		TIM2->SR = 0;
		
	/* Fin timer TIM2 */

	/* Timer TIM3 -> Para el modo melodía */
		TIM3->CR1 = 0x0000;
		TIM3->CR2 = 0x0000;
		TIM3->SMCR = 0x0000;
		
		
		TIM3->PSC = 31999;
		TIM3->CNT = 0; 
		TIM3->ARR = 0xFFFF;
		TIM3->CCR1 = 1000;
		
		TIM3->DIER = 0x0002; 
		
		TIM3->CCMR1 = 0x0000; 
		TIM3->CCER = 0x0000;
		
		TIM3->EGR |= 0x0001;
		TIM3->SR = 0; 
		TIM3->CR1 |= 0x0001;

	/* Fin timer TIM3 */


	/* Timer TIM4 -> Espera de los 10ms al actualizar las notas */
		TIM4->CR1 = 0x0000; 
		TIM4->CR2 = 0x0000; 
		TIM4->SMCR = 0x0000;
		
		TIM4->PSC = 7;
		TIM4->CNT = 0; 
		TIM4->ARR = 0xFFFF;
		TIM4->CCR1 = TIM4CCR1; 

		TIM4->DIER = (1<<2); 
		
		TIM4->CCMR1 = 0x0000; 
		TIM4->CCER = 0x0000;
		
		TIM4->EGR |= 0x0001;
		TIM4->SR = 0; 
		TIM4->CR1 |= 0x0001;
	
	/* Fin timer TIM4 */

	//MODER de las teclas
	GPIOA->MODER &= ~(1 << (11 * 2));			//DO
	GPIOA->MODER &= ~(1 << (11 * 2 + 1));	//DO
	GPIOA->MODER &= ~(1 << (12 * 2));			//RE
	GPIOA->MODER &= ~(1 << (12 * 2 + 1));	//RE
	GPIOB->MODER &= ~(1 << (2 * 2));			//MI
	GPIOB->MODER &= ~(1 << (2 * 2 + 1));	//MI
	GPIOC->MODER &= ~(1 << (12 * 2));			//FA
	GPIOC->MODER &= ~(1 << (12 * 2 + 1));	//FA
	GPIOC->MODER &= ~(1 << (13 * 2));			//SOL
	GPIOC->MODER &= ~(1 << (13 * 2 + 1));	//SOL
	GPIOD->MODER &= ~(1 << (2 * 2));			//LA
	GPIOD->MODER &= ~(1 << (2 * 2 + 1));	//LA
	GPIOH->MODER &= ~(1 << (1 * 2));			//SI
	GPIOH->MODER &= ~(1 << (1 * 2 + 1));	//SI
	
	
	//PA0 como entrada digital y como fuente de interrupción por flanco de subida
	GPIOA->MODER &= ~(1 << (0*2 +1));
  GPIOA->MODER &= ~(1 << (0*2));

	SYSCFG->EXTICR[0] = 0;
	EXTI->IMR |= 0x01; 
	EXTI->RTSR |= 0x01;
	EXTI->FTSR &= ~(0x01);
	NVIC->ISER[0] |= (1 << 6); 
	
	
	//PA5  salida del TIM2 (función alternativa 10)
	GPIOA->MODER |= (1 << (5 *2 + 1));
	GPIOA->MODER &= ~(1 << (5 *2));
	
	GPIOA->AFR[0] |= (1 << 5 * 4 );
	
	//Asignar el modo Pull-up (01) a las teclas
	/*GPIOA->PUPDR |= (1 << (11 * 2));			//DO
	GPIOA->PUPDR &= ~(1 << (11 * 2 + 1));	//DO
	GPIOA->PUPDR |= (1 << (12 * 2));			//RE
	GPIOA->PUPDR &= ~(1 << (12 * 2 + 1));	//RE
	GPIOB->PUPDR |= (1 << (2 * 2));				//MI
	GPIOB->PUPDR &= ~(1 << (2 * 2 + 1));	//MI
	GPIOC->PUPDR |= (1 << (12 * 2));			//FA
	GPIOC->PUPDR &= ~(1 << (12 * 2 + 1));	//FA
	GPIOC->PUPDR |= (1 << (13 * 2));			//SOL
	GPIOC->PUPDR &= ~(1 << (13 * 2 + 1));	//SOL
	GPIOD->PUPDR |= (1 << (2 * 2));				//LA
	GPIOD->PUPDR &= ~(1 << (2 * 2 + 1));	//LA
	GPIOH->PUPDR |= (1 << (1 * 2));				//SI
	GPIOH->PUPDR &= ~(1 << (1 * 2 + 1));	//SI
	*/
	//Habilitación de la interrupción del TIM2, del TIM3 y del TIM4
	NVIC->ISER[0] |= (1 << 28); 
	NVIC->ISER[0] |= (1 << 29); 
	NVIC->ISER[0] |= (1 << 30); 	

	//Habilición del DAC
	DAC->CR = 0x00000001; //El moder viene ya hecho por las funciones generadas por CUBE
	
	//USART
	HAL_UART_Receive_IT(&huart1, texto, 1);
	//notas
	int rec_do=DO,rec_re=RE,rec_mi=MI,rec_fa=FA,rec_sol=SOL,rec_la=LA,rec_si=SI;
	int dac_do=DOR,dac_re=RER,dac_mi=MIR,dac_fa=FAR,dac_sol=SOLR,dac_la=LAR,dac_si=SIR;
	
	
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
		
		
		if(changeDacTone){ //cambia la posicion del vector seno
			changeDacTone = 0;
			if(++posOnda > 19) posOnda = 0;
			if(ccrForTimDAC){
				DAC->DHR8R1 = seno[posOnda];
				TIM4->CCR2 += ccrForTimDAC;//ccrForTimDAC;
			}else{
				DAC->DHR8R1 = 0;
				TIM4->CCR2 += 1000;
			}
		}
		
		if(modo){ //Modo melodía 
			
			if(changeMode){ //Cambiado a modo piano
				BSP_LCD_GLASS_Clear();
				BSP_LCD_GLASS_DisplayString((unsigned char *)" PIANO");
				//Desactivamos los timers si salimos
				TIM3->CR1 = 0x0000;
				TIM2->CR1 = 0x0000;
					
				posSong = 0;
				changeMode = 0;
				modo = 0;
				ccrForTimDAC = 0;
				
			}else{
				
				
				if((TIM4->SR&0x0002) != 0){ // si han pasado 10 ms
					
					//Limpiamos flags
						TIM4->CCR1 += TIM4CCR1;
						TIM4->SR &= ~(0x0002);
					
				//Actualizamos los arrays de las notas 
						tecla_do[2] = tecla_do[1];
						tecla_do[1] = tecla_do[0];
						
						tecla_re[2] = tecla_re[1];
						tecla_re[1] = tecla_re[0];
						
						tecla_mi[2] = tecla_mi[1];
						tecla_mi[1] = tecla_mi[0];
						
						tecla_do[0] = ((GPIOA->IDR & (1 << (11))) != 0);	//DO
						tecla_re[0] = ((GPIOA->IDR & (1 << (12))) != 0);	//RE
						tecla_mi[0] = ((GPIOB->IDR & (1 << (2))) != 0);		//MI
				
				//Seleccion de melodia por tres pines 
						if (flanco_bajo(tecla_do)){
							BSP_LCD_GLASS_Clear();
							BSP_LCD_GLASS_DisplayString((unsigned char *)" MELO1");
							
							notas = cancion1;
							duraciones = duraciones1;
							nota_limite = 35;
							posSong = 0;
							changeMelodyTone = 1;
							newsong = 1;
							
						}
						if (flanco_bajo(tecla_re)){
							BSP_LCD_GLASS_Clear();
							BSP_LCD_GLASS_DisplayString((unsigned char *)" MELO2");
							
							notas = cancion2;
							duraciones = duraciones2;
							nota_limite = 19; 
							posSong = 0;
							changeMelodyTone = 1;
							newsong = 1;
							
							
						}
						if (flanco_bajo(tecla_mi)){
							BSP_LCD_GLASS_Clear();
							BSP_LCD_GLASS_DisplayString((unsigned char *)" MELO3");
							
							notas = cancion3;
							duraciones = duraciones3;
							nota_limite = 19;
							posSong = 0;
							changeMelodyTone = 1;
							newsong = 1;
							
						}
					}
				
				
					if(changeMelodyTone){ //combio de nota en la melodia
					TIM3->CR1 = 0x0000;
					if(newsong){ //cancion nueva
						TIM3->CNT = 0;
						TIM3->CCR1 = 0;
						newsong = 0;
					}
					
					TIM3->CCR1 += duraciones[posSong];
					TIM3->CR1 |= 0x0001;
					
					TIM2->CR1 = 0x0000; //Paramos el TIM2 para cambiar su CCR1 o por si hay una pausa
					 
					if(notas[posSong] != PAUSA){ //nota distinto de silencio
						activateTIM2(notas[posSong]); //Se cambia el CCR por el que indique la nota 
					}
					//Para el DA, el 80 viene por la relación entre los timers, porque son escalados distintos
					ccrForTimDAC = (notas[posSong] / 80);
						
					
					if( (++posSong) > nota_limite )
								posSong = 0;
					
					changeMelodyTone = 0;
					posOnda = 0;
					changeDacTone = 1; //forzamos actualización del DAC
				}
						
			}

		}else{ //Modo piano
			if (texto[0] != 0){

				if (texto[0] == '3'){
					texto[0] = 0;
					rec_do=DO3;rec_re=RE3;rec_mi=MI3;rec_fa=FA3;rec_sol=SOL3;rec_la=LA3;rec_si=SI3;
					dac_do=DO3R;dac_re=RE3R;dac_mi=MI3R;dac_fa=FA3R;dac_sol=SOL3R;dac_la=LA3R;dac_si=SI3R;
				}
				if (texto[0] == '4'){
					texto[0] = 0;
					rec_do=DO;rec_re=RE;rec_mi=MI;rec_fa=FA;rec_sol=SOL;rec_la=LA;rec_si=SI;
					dac_do=DOR;dac_re=RER;dac_mi=MIR;dac_fa=FAR;dac_sol=SOLR;dac_la=LAR;dac_si=SIR;
				} 
			}
			
			if(changeMode){ //Cambiamos a modo melodía
				BSP_LCD_GLASS_Clear();
				
				BSP_LCD_GLASS_DisplayString((unsigned char *)" MELO1");
				
				
				TIM3->CR1 = 0x0000;
				TIM3->CNT = 0;
				TIM3->CCR1 = 0;
		
				
				posSong = 0;//Reseteamos la melodia al principio
				changeMelodyTone = 1;
				changeMode = 0;
				modo = 1;
				
				notas = cancion1;
				duraciones = duraciones1;
				nota_limite = 35;
				
			}else{
				
					if((TIM4->SR&0x0002) != 0){ // si han pasado 10ms
						//limpiamos flags
						TIM4->CCR1 += TIM4CCR1;
						TIM4->SR &= ~(0x0002); 
						
					//asignamos valores al array de cada nota
						tecla_do[2] = tecla_do[1];
						tecla_do[1] = tecla_do[0];
						
						tecla_re[2] = tecla_re[1];
						tecla_re[1] = tecla_re[0];
						
						tecla_mi[2] = tecla_mi[1];
						tecla_mi[1] = tecla_mi[0];
						
						tecla_fa[2] = tecla_fa[1];
						tecla_fa[1] = tecla_fa[0];
						
						tecla_sol[2] = tecla_sol[1];
						tecla_sol[1] = tecla_sol[0];
						
						tecla_la[2] = tecla_la[1];
						tecla_la[1] = tecla_la[0];
						
						tecla_si[2] = tecla_si[1];
						tecla_si[1] = tecla_si[0];
					
						
						tecla_do[0] = ((GPIOA->IDR & (1 << (11))) != 0);	//DO
						tecla_re[0] = ((GPIOA->IDR & (1 << (12))) != 0);	//RE
						tecla_mi[0] = ((GPIOB->IDR & (1 << (2))) != 0);		//MI
						tecla_fa[0] = ((GPIOC->IDR & (1 << (12))) != 0);	//FA
						tecla_sol[0] = ((GPIOC->IDR & (1 << (13))) != 0);	//SOL
						tecla_la[0] = ((GPIOD->IDR & (1 << (2))) != 0);		//LA
						tecla_si[0] = ((GPIOH->IDR & (1 << (1))) != 0);		//SI


						if (flanco_bajo(tecla_do)){
							BSP_LCD_GLASS_Clear();
							BSP_LCD_GLASS_DisplayString((unsigned char *)"    DO");
							
							ccrForTimDAC = dac_do;
							changeDacTone = 1;
							
							activateTIM2(rec_do);
						}
						else if (flanco_alto(tecla_do)){
							BSP_LCD_GLASS_Clear();
							BSP_LCD_GLASS_DisplayString((unsigned char *)" PIANO");
							TIM2->CR1 = 0x0000;
							DAC->DHR8R1 = 0; ccrForTimDAC = 0;
						}
						
						if (flanco_bajo(tecla_re)){
							BSP_LCD_GLASS_Clear();
							BSP_LCD_GLASS_DisplayString((unsigned char *)"    RE");
							
							ccrForTimDAC = dac_re;
							changeDacTone = 1;

							activateTIM2(rec_re);
						}
						else if (flanco_alto(tecla_re)){
							BSP_LCD_GLASS_Clear();
							BSP_LCD_GLASS_DisplayString((unsigned char *)" PIANO");
							TIM2->CR1 = 0x0000;
							DAC->DHR8R1 = 0; ccrForTimDAC = 0;
						}
						
						if (flanco_bajo(tecla_mi)){
							BSP_LCD_GLASS_Clear();
							BSP_LCD_GLASS_DisplayString((unsigned char *)"    MI");
							
							ccrForTimDAC = dac_mi;
							changeDacTone = 1;
							
							activateTIM2(rec_mi);
						}
						else if (flanco_alto(tecla_mi)){
							BSP_LCD_GLASS_Clear();
							BSP_LCD_GLASS_DisplayString((unsigned char *)" PIANO");
							TIM2->CR1 = 0x0000;
							DAC->DHR8R1 = 0; ccrForTimDAC = 0;
						}
						if (flanco_bajo(tecla_fa)){
							BSP_LCD_GLASS_Clear();
							BSP_LCD_GLASS_DisplayString((unsigned char *)"    FA");
							
							ccrForTimDAC = dac_fa;
							changeDacTone = 1;
							activateTIM2(rec_fa);
						}
						else if (flanco_alto(tecla_fa)){
							BSP_LCD_GLASS_Clear();
							BSP_LCD_GLASS_DisplayString((unsigned char *)" PIANO");
							TIM2->CR1 = 0x0000;
							DAC->DHR8R1 = 0; ccrForTimDAC = 0;
						}
						
						if (flanco_bajo(tecla_sol)){
							BSP_LCD_GLASS_Clear();
							BSP_LCD_GLASS_DisplayString((unsigned char *)"   SOL");
							
							ccrForTimDAC = dac_sol;
							changeDacTone = 1;
							activateTIM2(rec_sol);
						}
						else if (flanco_alto(tecla_sol)){
							BSP_LCD_GLASS_Clear();
							BSP_LCD_GLASS_DisplayString((unsigned char *)" PIANO");
							TIM2->CR1 = 0x0000;
							DAC->DHR8R1 = 0; ccrForTimDAC = 0;
						}
						
						if (flanco_bajo(tecla_la)){
							BSP_LCD_GLASS_Clear();
							BSP_LCD_GLASS_DisplayString((unsigned char *)"    LA");
							
							ccrForTimDAC = dac_la;
							changeDacTone = 1;
							activateTIM2(rec_la);
						}
						else if (flanco_alto(tecla_la)){
							BSP_LCD_GLASS_Clear();
							BSP_LCD_GLASS_DisplayString((unsigned char *)" PIANO");
							TIM2->CR1 = 0x0000;
							DAC->DHR8R1 = 0; ccrForTimDAC = 0;
						}
						
						if (flanco_bajo(tecla_si)){
							BSP_LCD_GLASS_Clear();
							BSP_LCD_GLASS_DisplayString((unsigned char *)"    SI");
							
							ccrForTimDAC = dac_si;
							changeDacTone = 1;
							activateTIM2(rec_si);
						}
						else if (flanco_alto(tecla_si)){
							BSP_LCD_GLASS_Clear();
							BSP_LCD_GLASS_DisplayString((unsigned char *)" PIANO");
							TIM2->CR1 = 0x0000;
							DAC->DHR8R1 = 0; ccrForTimDAC = 0;
						}
	
				}//fin if 10 ms
			}//fin else
			
		
	}
    /* USER CODE END WHILE */

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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /**Configure the main internal regulator output voltage 
  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  /**Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_LSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL6;
  RCC_OscInitStruct.PLL.PLLDIV = RCC_PLL_DIV3;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /**Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_LCD;
  PeriphClkInit.LCDClockSelection = RCC_RTCCLKSOURCE_LSI;

  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC_Init(void)
{

  /* USER CODE BEGIN ADC_Init 0 */

  /* USER CODE END ADC_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC_Init 1 */

  /* USER CODE END ADC_Init 1 */
  /**Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion) 
  */
  hadc.Instance = ADC1;
  hadc.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV1;
  hadc.Init.Resolution = ADC_RESOLUTION_12B;
  hadc.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc.Init.EOCSelection = ADC_EOC_SEQ_CONV;
  hadc.Init.LowPowerAutoWait = ADC_AUTOWAIT_DISABLE;
  hadc.Init.LowPowerAutoPowerOff = ADC_AUTOPOWEROFF_DISABLE;
  hadc.Init.ChannelsBank = ADC_CHANNELS_BANK_A;
  hadc.Init.ContinuousConvMode = DISABLE;
  hadc.Init.NbrOfConversion = 1;
  hadc.Init.DiscontinuousConvMode = DISABLE;
  hadc.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc.Init.DMAContinuousRequests = DISABLE;
  if (HAL_ADC_Init(&hadc) != HAL_OK)
  {
    Error_Handler();
  }
  /**Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time. 
  */
  sConfig.Channel = ADC_CHANNEL_11;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_4CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC_Init 2 */

  /* USER CODE END ADC_Init 2 */

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
  /**DAC Initialization 
  */
  hdac.Instance = DAC;
  if (HAL_DAC_Init(&hdac) != HAL_OK)
  {
    Error_Handler();
  }
  /**DAC channel OUT1 config 
  */
  sConfig.DAC_Trigger = DAC_TRIGGER_NONE;
  sConfig.DAC_OutputBuffer = DAC_OUTPUTBUFFER_ENABLE;
  if (HAL_DAC_ConfigChannel(&hdac, &sConfig, DAC_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN DAC_Init 2 */

  /* USER CODE END DAC_Init 2 */

}

/**
  * @brief LCD Initialization Function
  * @param None
  * @retval None
  */
static void MX_LCD_Init(void)
{

  /* USER CODE BEGIN LCD_Init 0 */

  /* USER CODE END LCD_Init 0 */

  /* USER CODE BEGIN LCD_Init 1 */

  /* USER CODE END LCD_Init 1 */
  hlcd.Instance = LCD;
  hlcd.Init.Prescaler = LCD_PRESCALER_1;
  hlcd.Init.Divider = LCD_DIVIDER_31;
  hlcd.Init.Duty = LCD_DUTY_1_4;
  hlcd.Init.Bias = LCD_BIAS_1_3;
  hlcd.Init.VoltageSource = LCD_VOLTAGESOURCE_INTERNAL;
  hlcd.Init.Contrast = LCD_CONTRASTLEVEL_0;
  hlcd.Init.DeadTime = LCD_DEADTIME_0;
  hlcd.Init.PulseOnDuration = LCD_PULSEONDURATION_4;
  hlcd.Init.MuxSegment = LCD_MUXSEGMENT_DISABLE;
  hlcd.Init.BlinkMode = LCD_BLINKMODE_OFF;
  hlcd.Init.BlinkFrequency = LCD_BLINKFREQUENCY_DIV32;
  if (HAL_LCD_Init(&hlcd) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN LCD_Init 2 */

  /* USER CODE END LCD_Init 2 */

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

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 0;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

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

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 0;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 0;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */

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

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 0;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 0;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  if (HAL_TIM_Base_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim4, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */

}

/**
  * @brief TS Initialization Function
  * @param None
  * @retval None
  */
static void MX_TS_Init(void)
{

  /* USER CODE BEGIN TS_Init 0 */

  /* USER CODE END TS_Init 0 */

  /* USER CODE BEGIN TS_Init 1 */

  /* USER CODE END TS_Init 1 */
  /* USER CODE BEGIN TS_Init 2 */

  /* USER CODE END TS_Init 2 */

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 9600;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

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
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin : SOL_Pin */
  GPIO_InitStruct.Pin = SOL_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(SOL_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : SI_Pin */
  GPIO_InitStruct.Pin = SI_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(SI_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : PA0 */
  GPIO_InitStruct.Pin = GPIO_PIN_0;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : LA_Pin */
  GPIO_InitStruct.Pin = LA_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(LA_GPIO_Port, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
void activateTIM2(int newccr){ //funcion que activa el timer
			ccr = newccr;
			TIM2->CCR1 = newccr;
			TIM2->CR1 |= 0x0001;
	
}
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
 HAL_UART_Receive_IT(huart, texto, 1); // Vuelve a iniciar Rx por haber acabado el buffer
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
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
