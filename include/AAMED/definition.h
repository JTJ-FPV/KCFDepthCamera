#pragma once


#define FLED_EDGECONTOURS               0x21
#define FLED_FSAARCCONTOURS             0x22
#define FLED_FSALINES                   0x23
#define FLED_FSAARCSLINKMATRIX          0x24
#define FLED_DPCONTOURS                 0x25


#define FLED_GROUPING_IBmA1_IAnB1      0x00
#define FLED_GROUPING_FBmA1_FAnB1      0x00
#define FLED_GROUPING_FBmA1_CAnB1      0x01
#define FLED_GROUPING_CBmA1_FAnB1      0x10
#define FLED_GROUPING_CBmA1_CAnB1      0x11
#define FLED_GROUPING_CAnB1            0x01
#define FLED_GROUPING_CBmA1            0x10




#define FLED_SEARCH_LINKING            true
#define FLED_SEARCH_LINKED             false



#define ADAPT_APPROX_CONTOURS 1

#define DEFINITE_ERROR_BOUNDED 1

#define FASTER_ELLIPSE_VALIDATION 0


#define NONE_CLUSTER_METHOD   0
#define PRASAD_CLUSTER_METHOD 1
#define OUR_CLUSTER_METHOD    2
#define SELECT_CLUSTER_METHOD PRASAD_CLUSTER_METHOD

#define DETAIL_BREAKDOWN
