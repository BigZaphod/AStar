/*
 Copyright (c) 2012, Sean Heber. All rights reserved.
 
 Redistribution and use in source and binary forms, with or without
 modification, are permitted provided that the following conditions are met:
 
 1. Redistributions of source code must retain the above copyright
 notice, this list of conditions and the following disclaimer.
 
 2. Redistributions in binary form must reproduce the above copyright notice,
 this list of conditions and the following disclaimer in the documentation
 and/or other materials provided with the distribution.
 
 3. Neither the name of Sean Heber nor the names of its contributors may
 be used to endorse or promote products derived from this software without
 specific prior written permission.
 
 THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 DISCLAIMED. IN NO EVENT SHALL SEAN HEBER BE LIABLE FOR ANY DIRECT,
 INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
 LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE
 OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF
 ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef AStar_h
#define AStar_h

#include <stdlib.h>

typedef struct __ASNeighborList *ASNeighborList;
typedef struct __ASPath *ASPath;

typedef struct {
    size_t  nodeSize;
    int     (*nodeComparator)(void *node1, void *node2, void *context);             // must return a sort order for the nodes (-1, 0, 1)
    void    (*nodeNeighbors)(ASNeighborList neighbors, void *node, void *context);  // add nodes to the neighbor list if they are connected to this node
    float   (*pathCostHeuristic)(void *fromNode, void *toNode, void *context);      // estimated cost to travel from the first node to the second node
} ASPathNodeSource;

void ASNeighborListAdd(ASNeighborList neighbors, void *node, float edgeCost);

ASPath ASPathCreate(const ASPathNodeSource *nodeSource, void *context, void *startNode, void *goalNode);
void ASPathDestroy(ASPath path);

ASPath ASPathCopy(ASPath path);

float ASPathGetCost(ASPath path);
size_t ASPathGetCount(ASPath path);
void *ASPathGetNode(ASPath path, size_t index);

#endif
