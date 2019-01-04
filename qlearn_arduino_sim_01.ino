/*
 * RL Algos: Q_Learning
 * 2D Gridworld Example
 * 
 * Policy: Epsilon-Greedy Action
 * Rewards: +1 forward Movement
 *          +1 no Movement
 *          -1 backward Movement
 *          measured by 2 Optical Encoders
 * Action: 2 Servo robot arm with a stick
 * 
 * Q-learning algorithm:
 * (1) Init Tables              __
 * (2) Choose an Action           | Loop for 
 * (3) Find Qmax of Next State    | good Q-Table
 * (4) Do Bellman Update        __|
 * 
 * HW: Servo SG90, Encoder LM393
 * Author: Roboball 11/2018
 */ 
#include<stdio.h>
#include<stdlib.h>
#include<string.h>
#include<time.h>
#include<assert.h>
#include <Servo.h>

//////////////////////////////
// init RL globals
//////////////////////////////
#define ITERATIONS 1000 // set number of iterations, e.g. 1000
#define ROWS 5 // grid size, number of rows, e.g 5
#define COLS 5 // grid size, number of columns, e.g 5
#define STRING_LEN 10 // string length for printing, e.g 10
#define MAX_ACTIONS 4 // max number of next actions
#define DIM_STATES 2 // number of state dimensions
#define START_ROW 0 // set row of start state, e.g 0
#define START_COL 0 // set column of start state, e.g 0

const char *actions[] = {"up", "right", "down", "left"}; // actions
static double gamma = 0.9; // discount factor, e.g. 0.9, set low for reward only
static double alpha = 0.9; // learning factor, e.g. 0.85, set high for exploration
volatile double epsilon = 0.99; // epsilon-greedy factor (linear decay)
const double eps_decay = 0.99; // eps decay factor (set high for big envs)
const double eps_minimum = 0.1; // minimum epsilon random action, e.g. 0.1

double R[ROWS][COLS]; // reward table
double Q[ROWS][COLS]; // q table
char P[ROWS][COLS][STRING_LEN]; // policy table

// init struct for a state
typedef struct ST_State{
    int r; // row
    int c; // col
} State;

// init struct for array of actions
typedef struct ST_ActionStates{
    int v1[MAX_ACTIONS]; // case: corner up-left  
    int v2[MAX_ACTIONS]; // case: corner up-right
    int v3[MAX_ACTIONS]; // case: corner down-right
    int v4[MAX_ACTIONS]; // case: corner down-left
    int v5[MAX_ACTIONS]; // case: rim up
    int v6[MAX_ACTIONS]; // case: rim right
    int v7[MAX_ACTIONS]; // case: rim down
    int v8[MAX_ACTIONS]; // case: rim left
    int v9[MAX_ACTIONS]; // standard case, action order: (North, East, South, West)
}StateAction;

// init tuple for maximum
typedef struct ST_MaxTuple{
 double max_val; 
 int max_idx;   
} MaxTuple;

//////////////////////////////
// init further RLs
//////////////////////////////
StateAction state_actions[DIM_STATES] = {
    {        
    .v1 = { 0, 0, 1, 0 },
    .v2 = { 0, 0, 1, 0 },
    .v3 = { -1, 0, 0, 0 },
    .v4 = { -1, 0, 0, 0 },
    .v5 = { 0, 0, 1, 0 },
    .v6 = { -1, 0, 1, 0 },
    .v7 = { -1, 0, 0, 0 },
    .v8 = { -1, 0, 1, 0 },
    .v9 = { -1, 0, 1, 0 },  
    },
    { 
    .v1 = { 0, 1, 0, 0 },
    .v2 = { 0, 0, 0, -1 },
    .v3 = { 0, 0, 0, -1 },
    .v4 = { 0, 1, 0, 0 },
    .v5 = { 0, 1, 0, -1 },
    .v6 = { 0, 0, 0, -1 },
    .v7 = { 0, 1, 0, -1 },
    .v8 = { 0, 1, 0, 0 },
    .v9 = { 0, 1, 0, -1 },    
    }, 
}; // init special cases, first rows then cols

State current_state = {.r = START_ROW, .c = START_COL}; // init start position at e.g. [0,0]
//  if(current_state.r >= ROWS || current_state.c >= COLS){
//      //printf("\nError, Start State is not within Q-table.\n"); 
//      //return -1;  
//  };
State next_states[MAX_ACTIONS]; // init next possible states
State next_state; // init next state (for action)
MaxTuple max; // init for Q maximum
MaxTuple eps_max; // init for epsilon maximum

double Q_current; // current q value for current state
double Q_new; // new q value for current state
double Q_max; // max q value from next state
double R_current; // current reward for current state
double Q_max_array[MAX_ACTIONS]; // vector for Q-values to find max
double eps_array[DIM_STATES] = {eps_minimum, 1.0}; // vector for epsilon to find max
double init_q = 0.0; // initial q value
double init_reward = 1.0; // initial reward
int init_action = 0; // init action: up
int action; // action for next state
int iter = 1; // current number of iterations
double rand_uni; // uniform random number
time_t t; // for pseudo random number

//////////////////////////////
// init Hardware
//////////////////////////////
// init rotary encoders
static int pin2 = 2;
static int pin3 = 3;
volatile int last_encoded = 0;
volatile long encoder_value = 0;
int last_MSB = 0;
int last_LSB = 0;
volatile int MSB = LOW;
volatile int LSB = LOW;
volatile int encoded;
volatile int sum;
volatile long pos_value = 0;
volatile long neg_value = 0;

// init servos
Servo arm_down;  
Servo arm_up;
// init constants
static int servo_01 = 9;
static int servo_02 = 10;
volatile int pos_up = 120;  // initial position servo 01 
volatile int pos_down = 0;  // initial position servo 02 
// manually adjust servo boundaries
static int low_bnd_01 = 50; // lower boundary, servo 01, arm up
static int up_bnd_01 = 120; // upper boundary, servo 01, arm up
static int low_bnd_02 = 30; // lower boundary, servo 02, arm down
static int up_bnd_02 = 120; // upper boundary, servo 02, arm down

//////////////////////////////
// init helper functions
//////////////////////////////

double rand_double() {
    /* uniform random number */
   return rand()/(double)RAND_MAX;
}

int rand_int(int max_rand){
    /* random number between 0 and max_rand-1 */
    return rand() % max_rand;
}

int fill_mat_const(double array[][COLS], double val){
    /* fills a matrix with a constant value */
    int i,j;
    for(i=0; i < ROWS; i++){
        for(j=0; j<COLS; ++j){
            array[i][j] = val;
        }   
    }
    return 0;
}

int fill_mat_string(char array[][COLS][STRING_LEN], int val){
    /* fills a matrix with a constant string */
    int i,j;
    for(i=0; i < ROWS; i++){
        for(j=0; j<COLS; ++j){
            strcpy(array[i][j], actions[val]); 
        }   
    }
    return 0;
}

int fill_Qvec(double vec[], State states[MAX_ACTIONS]){
    /* fill vector with Q-Values */
    for (int i = 0; i < MAX_ACTIONS; i++){
        vec[i] = Q[states[i].r][states[i].c];   
    }
    return 0;
}

void print_mat_double(double array[][COLS]){
    /* prints out a double matrix as strings */
    int i,j;
    char out_tmp[STRING_LEN];
    char output[STRING_LEN];
    for(i=0; i < ROWS; i++){
        for(j=0; j<COLS; ++j){
          dtostrf(array[i][j], 4, 2, out_tmp);
          snprintf(output, STRING_LEN, "%8s", out_tmp);
          Serial.print(output);
          memset(&out_tmp[0], 0, sizeof(output));
          memset(&output[0], 0, sizeof(output));
        }
//        printf("\n");
        Serial.println("");    
    }
//    printf("\n");
    Serial.println(""); 
}

void print_mat_string(char array[][COLS][STRING_LEN]){
    /* prints out a string matrix */
    int i,j;
    char output[STRING_LEN];
    for(i=0; i < ROWS; i++){
        for(j=0; j<COLS; ++j){
          snprintf(output, STRING_LEN, "%8s", array[i][j]);
//            printf("%8s ", array[i][j]);
            Serial.print(output);
        }
//        printf("\n");
        Serial.println("");    
    }
//    printf("\n");
    Serial.println(""); 
}

void print_vec(double vec[]){
    /* prints out a double vector */
//    printf("\nQVector: ( ");
    Serial.print("\nQVector: ( "); 
    for (int i = 0; i < MAX_ACTIONS; ++i){ 
//        printf("%3.2lf ", vec[i]);
        Serial.print(vec[i],2);
        Serial.print(" "); 
    }
//    printf(")");
    Serial.print(")"); 
}

int next_actions(State current_state, State next_states[MAX_ACTIONS], StateAction state_actions[DIM_STATES]){
    /* return next possible actions by array of structs */
    int i;
    Serial.print("\nCurrent State: (");
    Serial.print(current_state.r);
    Serial.print(","); 
    Serial.print(current_state.c);
    Serial.println(")"); 
    Serial.print("\nPossible Next States:\n");
    // loop over all actions by order: (North, East, South, West)
    for (i = 0; i < MAX_ACTIONS; i++){    
        // check 9 cases: 4 corner, 4 rim cases, 1 standard case
        if(current_state.r == 0 && current_state.c == 0){
            // case: corner up-left
            next_states[i].r = state_actions[0].v1[i]; // assign value to row
            next_states[i].c = state_actions[1].v1[i]; // assign value to col         
        }
        else if(current_state.r == 0 && current_state.c == COLS-1){  
            // case: corner up-right
            next_states[i].r = state_actions[0].v2[i]; // assign value to row
            next_states[i].c = COLS -1 + state_actions[1].v2[i]; // assign value to col
        }
        else if(current_state.r == ROWS-1 && current_state.c == COLS-1){
            // case: corner down-right
            next_states[i].r = ROWS -1 + state_actions[0].v3[i]; // assign value to row
            next_states[i].c = COLS -1 + state_actions[1].v3[i]; // assign value to col       
        }
        else if(current_state.r == ROWS-1 && current_state.c == 0){        
            // case: corner down-left
            next_states[i].r = ROWS -1 + state_actions[0].v4[i]; // assign value to row
            next_states[i].c = state_actions[1].v4[i]; // assign value to col    
        }
        else if(current_state.r == 0 && current_state.c != 0 && current_state.c != COLS-1){
            // case: rim up
            next_states[i].r = state_actions[0].v5[i]; // assign value to row
            next_states[i].c = current_state.c + state_actions[1].v5[i]; // assign value to col 
        }
        else if(current_state.c == COLS-1 && current_state.r != 0 && current_state.r != ROWS-1){
            //  case: rim right
            next_states[i].r = current_state.r + state_actions[0].v6[i]; // assign value to row
            next_states[i].c = COLS -1 + state_actions[1].v6[i]; // assign value to col       
        }
        else if(current_state.r == ROWS-1 && current_state.c != 0 && current_state.c != COLS-1){
            //  case: rim down
            next_states[i].r = ROWS -1 + state_actions[0].v7[i]; // assign value to row
            next_states[i].c = current_state.c + state_actions[1].v7[i]; // assign value to col     
        }
        else if(current_state.c == 0 && current_state.r != 0 && current_state.r != ROWS-1){
            //  case: rim left
            next_states[i].r = current_state.r + state_actions[0].v8[i]; // assign value to row
            next_states[i].c = state_actions[1].v8[i]; // assign value to col        
        }
         else if(current_state.r != 0 && current_state.r != ROWS-1 && current_state.c != 0 && current_state.c != COLS-1 ){
            // standard case
            next_states[i].r = current_state.r + state_actions[0].v9[i]; // assign value to row
            next_states[i].c = current_state.c + state_actions[1].v9[i]; // assign value to col     
        }
        else{
              Serial.print("\nError, no next states found!!\n");
              return -1; 
        }
        Serial.print(actions[i]);
        Serial.print(": (");
        Serial.print(next_states[i].r);
        Serial.print(","); 
        Serial.print(next_states[i].c);
        Serial.println(")"); 
    }
    return 0;
}

int find_max_vec(double array[], MaxTuple *max, int vec_len){
    /* find maximum value and index in vector */
    max->max_val = array[0];
    max->max_idx = 0;

    for(int r = 0; r < vec_len; r++){
        if (array[r] > max->max_val){
            max->max_val = array[r];
            max->max_idx = r;
        }
    }
    Serial.print("\nMax: ");
    Serial.print(max->max_val);
    Serial.print(", Idx:"); 
    Serial.print(max->max_idx);
    Serial.println(""); 
    return 0; 
}

void isr_update(){
  /* interrupt routine to read 2 optical encoders */
  MSB = digitalRead(pin2); //MSB = most significant bit
  LSB = digitalRead(pin3); //LSB = least significant bit

  encoded = (MSB << 1) |LSB; //converting the 2 pin value to single number
  sum  = (last_encoded << 2) | encoded; //adding it to the previous encoded value

  if(sum == 0b1101 || sum == 0b0100 || sum == 0b0010 || sum == 0b1011){ 
    encoder_value ++;
    pos_value ++;
  }
  if(sum == 0b1110 || sum == 0b0111 || sum == 0b0001 || sum == 0b1000){ 
    encoder_value --;
    neg_value --;
  }
  last_encoded = encoded; //store this value for next time
}

//////////////////////////////
// init setup and main
//////////////////////////////

void setup() {
  Serial.begin(9600); // start the serial monitor link
  Serial.print("\n/////////////////////\nStart with Setup\n/////////////////////\n");
  
  //////////////////////////////
  // setup RL gridworld
  //////////////////////////////
  srand((unsigned) time(&t)); // init random number generator
  // init tables
  Serial.print("\n==== Init Tables ====\n");
  fill_mat_const(Q, init_q); // init Q-table with zeros
  Serial.print("\nQ-table\n"); 
  print_mat_double(Q);
  // init R-table 
  fill_mat_const(R, init_reward);
  R[1][4] = 100.0;
  R[2][4] = 50.0;
  R[1][3] = 25.0;
  Serial.print("Reward-table\n");
  print_mat_double(R);
  // init Policy Table with north actions
  fill_mat_string(P, init_action);
  //strcpy(P[1][3], actions[init_action]); // Update Policy Table 
  Serial.print("Action-table\n");
  print_mat_string(P);

  // check possible next states for start state: Q[0][0] 
  Serial.print("=== Check for Start State ===\n");
  next_actions(current_state, next_states, state_actions);

  //////////////////////////////
  // setup Hardware
  //////////////////////////////
  // setup rotary encoders
  pinMode(pin2, INPUT_PULLUP); // pulled HIGH to the logic voltage (5V or 3.3V for most cases)
  pinMode(pin3, INPUT_PULLUP); // pulled HIGH to the logic voltage (5V or 3.3V for most cases)
  attachInterrupt(digitalPinToInterrupt(pin2), isr_update, CHANGE);
  attachInterrupt(digitalPinToInterrupt(pin3), isr_update, CHANGE);
  // setup servos
  arm_up.attach(servo_01);  
  arm_down.attach(servo_02);
  arm_up.write(pos_up); // go to init position
  Serial.print("\n/////////////////////\nSetup finished\n/////////////////////\n");
}


void loop() {

  Serial.print("\n\n==================================\n");  
  Serial.print("  Start with Iteration (");  
  Serial.print(iter);  
  Serial.print("|");
  Serial.print(ITERATIONS);  
  Serial.print(")\n==================================\n");  

  // epsilon-greedy strategy with expontial decay
  Serial.print("\n=== Choose an Action ===\n"); 
  eps_array[1] = epsilon * eps_decay;
  find_max_vec(eps_array, &eps_max, DIM_STATES); // find max of epsilon
  epsilon = eps_max.max_val;
  rand_uni = rand_double();
  Serial.print("\nEpsilon: ");
  Serial.println(epsilon,2); 
  Serial.print("\nRandUni: ");
  Serial.println(rand_uni,2); 
  if(epsilon > rand_uni){ 
      // case: random action 
      action = rand_int(MAX_ACTIONS); // rand(number from 0 to 3)
      Serial.print("\nRandom Action: "); 
  }
  else{ 
      // case: greedy action 
      fill_Qvec(Q_max_array, next_states); // find Q-Values
      print_vec(Q_max_array);
      find_max_vec(Q_max_array, &max, MAX_ACTIONS); // find max of Q-Values
      action = max.max_idx;
      Serial.print("\nGreedy Action: ");
  }
  Serial.print(actions[action]);
  Serial.print(", ("); 
  Serial.print(action);
  Serial.println(")");  
  // set the next state
  next_state.r = next_states[action].r;
  next_state.c = next_states[action].c;

  // opt: show action in action table
  strcpy(P[current_state.r][current_state.c], actions[action]);
  Serial.print("\nAction-table\n"); 
  print_mat_string(P);

  // find Q_max of next state
  Serial.print("\n=== Find Qmax of Next State ===\n");
  next_actions(next_state, next_states, state_actions);
  fill_Qvec(Q_max_array, next_states); // find Q-Values
  print_vec(Q_max_array);
  find_max_vec(Q_max_array, &max, MAX_ACTIONS); // find max of Q-Values
  
  // assign values
  Q_max = max.max_val; 
  Q_current = Q[current_state.r][current_state.c]; 
  R_current = R[current_state.r][current_state.c];

  // do Bellman Update
  Q_new = Q_current + alpha * (R_current + gamma * Q_max - Q_current);
  Q[current_state.r][current_state.c] = Q_new; // Update Q-table
  Serial.print("\n=== Do Bellman Update ===\n");
  Serial.print("\nQ-Old: ");
  Serial.println(Q_current,2);
  Serial.print("\nQ-Update: ");
  Serial.println(Q_new,2);
  
  // set next state to current state
  current_state = next_state;

  Serial.print("\nUpdated Q-table\n");
  print_mat_double(Q);

  iter +=1; // increase iteration counter
  //delay(2000); // optional: time delay in ms
}
