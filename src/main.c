/*

The project "usart-interrupt" has been used as the base for this project.
Located in:
https://github.com/thomasgadner/MECH-B-4-ILV-Embedded-Systems/tree/master/code-examples/05_Digitale%20Kommunikation/usart-interrupt

*/

#include <stm32f0xx.h>
#include <stdbool.h>
#include <string.h>

//---------------------------------------------------------------------------------------------------------------------
//----------------------------------------Defines, structs and global variables----------------------------------------
//---------------------------------------------------------------------------------------------------------------------

#define APB_FREQ 48000000
#define AHB_FREQ 48000000

#define BAUDRATE 115200 // Baud rate set to 115200 baud per second

//------------------------------Button------------------------------
#define NUCLEO64_BUTTON 13
bool oldButtonState;
bool newButtonstate;

//------------------------------FIFO and UART------------------------------
#define FIFO_SIZE 64
#define FIFO_ERROR -1

#define USART2_RX_PIN 3 // PA3 is used as USART2_RX
#define USART2_TX_PIN 2 // PA2 is used as USART2_TX

typedef struct {
    uint8_t buffer[FIFO_SIZE];
    uint16_t head;
    uint16_t tail;
} Fifo_t;

volatile Fifo_t usart_rx_fifo;

char testString[] = {'t','e','s','t','\n'};

//------------------------------Game------------------------------
char playerField[100] = {0};
char enemyField[100] = {0};

typedef enum {
  NONE,
  HD_START,
  DH_START,
  HD_CS,
  DH_CS,
  HD_BOOM,
  DH_BOOM,
  ERROR
}message;

uint16_t currentMessage;
bool     fullMessageReceived;

char const strHD_START[]  = "HD_START";
char const strHD_CS[]     = "HD_CS";
char const strHD_BOOM[]   = "HD_BOOM";

typedef enum {
  IDLE,
  START,
  CS_GEN,
  FIELD_GEN,
  FIRE,
  EVAL_MOVE,
  CALIFORNIA,
  CS_CHECK
}state;


//-----------------------------------------------------------------------------------------------------
//----------------------------------------Function declarations----------------------------------------
//-----------------------------------------------------------------------------------------------------

void fifo_init(Fifo_t*);
int fifo_put(Fifo_t*, uint8_t);
int fifo_get(Fifo_t*, uint8_t*);

void SystemClock_Config(void);

void sendMessage(char*, int);

message enumerateMessage(Fifo_t*);
//------------------------------------------------------------------------------------
//----------------------------------------Main----------------------------------------
//------------------------------------------------------------------------------------
int main(void)
{
  SystemClock_Config(); // Configure the system clock to 48 MHz

  RCC->AHBENR     |= RCC_AHBENR_GPIOAEN;    // Enable GPIOA clock
  RCC->AHBENR     |= RCC_AHBENR_GPIOCEN;    // Enable GPIOC clock
  RCC->APB1ENR    |= RCC_APB1ENR_USART2EN; // Enable USART2 clock

  GPIOC->MODER    &= ~(0b11 << 2*NUCLEO64_BUTTON);

  GPIOA->MODER    |= 0b10 << (USART2_TX_PIN * 2);    // Set PA2 to Alternate Function mode
  GPIOA->AFR[0]   |= 0b0001 << (4 * USART2_TX_PIN); // Set AF for PA2 (USART2_TX)
  GPIOA->MODER    |= 0b10 << (USART2_RX_PIN * 2);    // Set PA3 to Alternate Function mode
  GPIOA->AFR[0]   |= 0b0001 << (4 * USART2_RX_PIN); // Set AF for PA3 (USART2_RX)

  USART2->BRR     = (APB_FREQ / BAUDRATE); // Set baud rate (requires APB_FREQ to be defined)
  USART2->CR1     |= 0b1 << 2;             // Enable receiver (RE bit)
  USART2->CR1     |= 0b1 << 3;             // Enable transmitter (TE bit)
  USART2->CR1     |= 0b1 << 0;             // Enable USART (UE bit)
  USART2->CR1     |= 0b1 << 5;             // Enable RXNE interrupt (RXNEIE bit)

  NVIC_SetPriorityGrouping(0);                               // Use 4 bits for priority, 0 bits for subpriority
  uint32_t uart_pri_encoding = NVIC_EncodePriority(0, 1, 0); // Encode priority: group 1, subpriority 0
  NVIC_SetPriority(USART2_IRQn, uart_pri_encoding);          // Set USART2 interrupt priority
  NVIC_EnableIRQ(USART2_IRQn);                               // Enable USART2 interrupt

  fifo_init((Fifo_t *)&usart_rx_fifo);                       // Init the FIFO

  uint32_t bytes_recv = 0;

//-----------------------------------------------------------------------------------------
//----------------------------------------Superloop----------------------------------------
//-----------------------------------------------------------------------------------------

  while (1)
  {
    newButtonstate = ((GPIOC->IDR) & (1<<NUCLEO64_BUTTON));

    if (newButtonstate != oldButtonState) {
        sendMessage(testString,sizeof(testString));
    }

    uint8_t byte;
    if (fifo_get((Fifo_t *)&usart_rx_fifo, &byte) == 0)
    {
      bytes_recv++; // count the Bytes Received by getting Data from the FIFO
    }

    oldButtonState = newButtonstate;
  }

}

//------------------------------------------------------------------------------------------------------------
//----------------------------------------Interrupt service routine(s)----------------------------------------
//------------------------------------------------------------------------------------------------------------
void USART2_IRQHandler(void)
{
  static int ret; // You can do some error checking
  if (USART2->ISR & USART_ISR_RXNE)
  {                                              // Check if RXNE flag is set (data received)
    uint8_t c = USART2->RDR;                     // Read received byte from RDR (this automatically clears the RXNE flag)
    
    if (c == '\n')
      fullMessageReceived = true;
    else
      fullMessageReceived = false;

    ret = fifo_put((Fifo_t *)&usart_rx_fifo, c); // Put incoming Data into the FIFO Buffer for later handling
  }
}

//----------------------------------------------------------------------------------------------------
//----------------------------------------Function definitions----------------------------------------
//----------------------------------------------------------------------------------------------------

void sendMessage(char* data, int size) {

  int count = size;
  while (count--) {

      while (!(USART2->ISR & USART_ISR_TXE))
        ;
      
      USART2->TDR = *data++;
  }
}

//----------------------------------------FIFO----------------------------------------
void fifo_init(Fifo_t* fifo) {
    fifo->head = 0;                              // Initialize head pointer to 0
    fifo->tail = 0;                              // Initialize tail pointer to 0
}

uint8_t fifo_is_empty(Fifo_t* fifo) {
    return (fifo->head == fifo->tail);          // FIFO is empty if head and tail are equal
}

uint8_t fifo_is_full(Fifo_t* fifo) {
    return ((fifo->head + 1) % FIFO_SIZE) == fifo->tail; // FIFO is full if incrementing head would equal tail
}

int fifo_put(Fifo_t* fifo, uint8_t data) {
    if (fifo_is_full(fifo)) {                   // Check if FIFO is full before inserting
        return -1;                              // Insertion failed (buffer full)
    }

    fifo->buffer[fifo->head] = data;            // Store data at current head position
    fifo->head = (fifo->head + 1) % FIFO_SIZE;  // Move head forward and wrap around if needed
    return 0;                                   // Insertion successful
}

int fifo_get(Fifo_t* fifo, uint8_t* data) {
    if (fifo_is_empty(fifo)) {                  // Check if FIFO is empty before reading
        return -1;                              // Read failed (buffer empty)
    }

    *data = fifo->buffer[fifo->tail];           // Retrieve data at current tail position
    fifo->tail = (fifo->tail + 1) % FIFO_SIZE;  // Move tail forward and wrap around if needed
    return 0;                                   // Read successful Return 0
}

//----------------------------------------Clock----------------------------------------
void SystemClock_Config(void)
{
  // Reset the Flash 'Access Control Register', and
  // then set 1 wait-state and enable the prefetch buffer.
  // (The device header files only show 1 bit for the F0
  //  line, but the reference manual shows 3...)
  FLASH->ACR      &= ~(FLASH_ACR_LATENCY_Msk | FLASH_ACR_PRFTBE_Msk);
  FLASH->ACR      |= (FLASH_ACR_LATENCY |
                      FLASH_ACR_PRFTBE);

  // activate the internal 48 MHz clock
  RCC->CR2        |= RCC_CR2_HSI48ON;

  // wait for clock to become stable before continuing
  while (!(RCC->CR2 & RCC_CR2_HSI48RDY))
    ;

  // configure the clock switch
  RCC->CFGR       = RCC->CFGR & ~RCC_CFGR_HPRE_Msk;
  RCC->CFGR       = RCC->CFGR & ~RCC_CFGR_PPRE_Msk;
  RCC->CFGR       = (RCC->CFGR & ~RCC_CFGR_SW_Msk) | (0b11 << RCC_CFGR_SW_Pos);

  // wait for clock switch to become stable
  while ((RCC->CFGR & RCC_CFGR_SWS) != (0b11 << RCC_CFGR_SWS_Pos))
    ;
}

//----------------------------------------Enumerate Message----------------------------------------
message enumerateMessage(Fifo_t* Fifo) {
  message msg = NONE;
  static uint32_t   recChecksum;
  static uint8_t    recBoomX;
  static uint8_t    recBoomY;
  static bool       recHM;
  
  uint8_t           cnt = 0;
  char              buf[FIFO_SIZE] = {0};

  while(msg == NONE) {
      fifo_get(Fifo,buf[cnt]);

      if(strcmp(buf,strHD_START))
        msg = HD_START;
      
      if(strcmp(buf,strHD_BOOM))
        msg = HD_BOOM;

      if(strcmp(buf,strHD_CS))
        msg = HD_CS;

      if(cnt >= FIFO_SIZE-1)
        return ERROR;
      cnt++;
  }
}