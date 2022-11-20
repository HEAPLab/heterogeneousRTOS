/******************************************************************************
*   Copyright 2021 Politecnico di Milano
*
*   Licensed under the Apache License, Version 2.0 (the "License");
*   you may not use this file except in compliance with the License.
*   You may obtain a copy of the License at
*
*      http://www.apache.org/licenses/LICENSE-2.0
*
*   Unless required by applicable law or agreed to in writing, software
*   distributed under the License is distributed on an "AS IS" BASIS,
*   WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
*   See the License for the specific language governing permissions and
*   limitations under the License.
*******************************************************************************/

/*
   32-bits Uniform random number generator U[0,1) lfsr113, based on the
   work of Pierre L'Ecuyer (http://www.iro.umontreal.ca/~lecuyer/myftp/papers/tausme2.ps)
*/
#include "simple_random.h"
/**** VERY IMPORTANT **** :
  The initial seeds z1, z2, z3, z4  MUST be larger than
  1, 7, 15, and 127 respectively.
****/

static UINT32_T z1, z2, z3, z4;

void random_set_seed(UINT32_T seed)
{
    z1 = 1+seed;
    z2 = 7+seed;
    z3 = 15+seed;
    z4 = 127+seed;
}

double random_get(void)
{
    UINT32_T b;
    b  = ((z1 << 6) ^ z1) >> 13;
    z1 = ((z1 & 4294967294U) << 18) ^ b;
    b  = ((z2 << 2) ^ z2) >> 27;
    z2 = ((z2 & 4294967288U) << 2) ^ b;
    b  = ((z3 << 13) ^ z3) >> 21;
    z3 = ((z3 & 4294967280U) << 7) ^ b;
    b  = ((z4 << 3) ^ z4) >> 12;
    z4 = ((z4 & 4294967168U) << 13) ^ b;
    return (z1 ^ z2 ^ z3 ^ z4) * 2.3283064365386963e-10;
}

void random_get_array(double a[], int len){
    int i;
    for(i = 0;i < len; i++){
        a[i] = random_get();  
    }
}
static int partition(double array[],int low,int high){
    int i,j;
    double pivot,temp;
    
    pivot = array[high];
    i=low;
    for(j=low;j<=high;j++){
        if(array[j]<pivot){
            temp=array[j];
            array[j]=array[i];
            array[i]=temp;
            i++;
        }
    }
    temp=array[i];
    array[i]=array[high];
    array[high]=temp;
            
    return i;
}
/**
 * @brief Quick sort recursive implementation
 * 
 * @param low start index for partitions creation
 * @param high end index for partitions creation
 */
static void quick_sort(double array[],int low,int high){

    if(low<high){
        int pivot;
        pivot=partition(array,low,high);
        quick_sort(array,low, pivot-1);
        quick_sort(array,pivot+1,high);
    }
    
}
void random_get_sarray(double a[], int len){
    int i;

    for(i = 0;i < len; i++){
        a[i] = random_get();
    }
    
    quick_sort(a,0,len-1);
    
}

void random_get_barray(int a[], int len){
    int i;
    for(i = 0;i < len; i++){
        a[i] = random_get() > 0.5 ? 1 : 0;  
    }
}
