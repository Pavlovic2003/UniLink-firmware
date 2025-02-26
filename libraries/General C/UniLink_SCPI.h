/*
 * SCPI_lib.h
 *
 *  Created on: Apr 8, 2024
 *      Author: kuban
 */

#ifndef INC_UNILINK_SCPI_H_
#define INC_UNILINK_SCPI_H_

#include <string.h>
#include <stdio.h>
#include <stdlib.h>

typedef enum paramTypes { OFF_P = 0, ON_P = 1, EVAL_P = 2, INT_P = 3, FLOAT_P = 4,  OTHER_P = 5 }ParamTypes;

typedef enum subWordTypes { function = 0, params = 1, class = 2 }SubWordTypes;

typedef struct subword
{
	SubWordTypes type;
	int functionIndex;
	ParamTypes paramType;
	float floatParam;
	int integerParam;
	char* otherParam;
}Subword;

typedef struct word
{
	int address;
	int classIndex;
	Subword** subwords;
	int subwordsCount;
}Word;

typedef struct function
{
	char* name;
	void (*run)(Subword**, int);
}Function;

typedef struct class
{
	char* name;
	Function* functions;
	int functionsLength;
}Class;

//Class* classList;
//int classLength;
//int defaultClassIndex;

char* ReformatString(char* chararr, int arrMaxSize);

Subword* generateSubwordn(char* subcommand, int length, Class* class);

Word* generateWordDirect(char* command);

void executeWord(Word* word);

void addFunction(char* name, void (*func)(Subword**, int), Class* class);

void addEmptyClass(char* name, int isDefault);

void addClass(Class* class, int isDefault);

#endif
