/*
 * SCPI_lib.c
 *
 *  Created on: Apr 8, 2024
 *      Author: kuban
 */

#include "UniLink_SCPI.h"

char* paramsList[3] = { "OFF", "ON", "? " };
int paramsLength = 3;

struct function *functionList;
int functionLength = 0;

void ReformatString(char* chararr, int arrMaxSize)
{
	for (int i = 0; i < arrMaxSize && chararr[i] != '\0'; i++)	// format into proper string so stdlib.h can be used
	{
		if (chararr[i] == '\r' || chararr[i] == '\n')
		{
			chararr[i] = '\0';
			return;
		}
	}
}

struct subword generateSubwordn(char* subcommand, int length)
{
	struct subword finalSubword = { .type = params, .function = 0, .integerParam = 0, .otherParam = NULL, .paramType = 0};

	for (int i = 0; i < functionLength; i++)
	{
		if (!strncmp(subcommand, functionList[i].name, length))
		{
			finalSubword.type = function;
			finalSubword.function = (enum functionTypes)i;
			return finalSubword;
		}
	}

	for (int i = 0; i < paramsLength; i++)
	{
		if (!strncmp(subcommand, paramsList[i], length))
		{
			finalSubword.paramType = (enum paramTypes)i;
			return finalSubword;
		}
	}

	int n;
	if ((n = atoi(subcommand)))
	{
		finalSubword.paramType = INT_P;
		finalSubword.integerParam = n;
		return finalSubword;
	}

	finalSubword.paramType = OTHER_P;
	finalSubword.otherParam = (char*) malloc(sizeof(char) * (length + 1));
	if(finalSubword.otherParam != NULL) strncpy(finalSubword.otherParam, subcommand, length);
	return finalSubword;
}

struct word generateWordDirect(char* command)
{
	struct word finalWord = { .address = -1, .subwords = NULL, .subwordsCount = 0 };

	char* currSymbol = command;
	int intermediateLength = 0;

	int isLast = 0;

	while (!isLast)
	{
		isLast = *currSymbol == '\0';
		switch (*currSymbol)
		{
			case ':'	:
			case '\0'	:
			case '?'	:

				if(intermediateLength == 0) break;
				if (finalWord.address == -1)
				{
					finalWord.address = atoi(currSymbol - intermediateLength);
				}

				else
				{
					finalWord.subwordsCount++;
					struct subword *intermediate = (struct subword*)realloc(finalWord.subwords, finalWord.subwordsCount * sizeof(struct subword));
					if (intermediate != NULL)
					{
						finalWord.subwords = intermediate;																				///??????
						finalWord.subwords[finalWord.subwordsCount - 1] = generateSubwordn(currSymbol - intermediateLength, intermediateLength);
					}
				}

				intermediateLength = 0;

				break;

			default:
				intermediateLength++;
				break;

		}

		if(*currSymbol == '?')
		{
			finalWord.subwordsCount++;
			struct subword* intermediate = (struct subword*)realloc(finalWord.subwords, finalWord.subwordsCount * sizeof(struct subword));
			if (intermediate != NULL)
			{
				finalWord.subwords = intermediate;																						///??????
				finalWord.subwords[finalWord.subwordsCount - 1] = generateSubwordn("?", 1);
			}
		}

		currSymbol += !isLast;
	}

	return finalWord;
}

void executeWord(struct word word)
{
	if (word.subwordsCount < 1) return;
	if (word.subwords == NULL) return;
	if (word.subwords[0].type != function) return;
	int listIndex = (int)word.subwords[0].function;
	functionList[listIndex].run(word.subwords + 1, word.subwordsCount - 1);
}

void addFunction(char *name, void (*func)(struct subword*, int))
{
	functionLength++;
	struct function newFunc = {.name = name, .run = func};
	functionList = (struct function*) realloc(functionList, functionLength * sizeof(struct function));
	functionList[functionLength - 1] = newFunc;
}
