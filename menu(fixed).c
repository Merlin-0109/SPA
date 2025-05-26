#include <stdio.h>
#include <stdlib.h>
#include <limits.h>
#include <string.h>
#include <windows.h>
#include <conio.h>
#include <time.h>
#include <stdbool.h>
#include <math.h>

#define MAX_VERTICES 100
#define MAX_EDGES 1000
#define NO_EDGE 0

#define GREEN   "\033[32m"
#define BLUE    "\033[34m"
#define YELLOW  "\033[33m"
#define CYAN    "\033[36m"
#define RESET   "\033[0m"
#define WHITE   "\033[37m"
#define MINT "\033[38;2;152;255;152m"
#define MINT_GREEN "\033[38;2;152;255;152m"
#define RED "\033[31m"
#define MINT_BLUE "\033[38;2;152;255;255m"
#define HEAP_MAX_SIZE MAX_VERTICES

typedef struct {
    int vertex;
    double distance;
} HeapNode;

typedef struct {
    HeapNode* nodes;
    int size;
    int capacity;
    int* pos; // Mảng để lưu vị trí hiện tại của một đỉnh trong heap
} MinHeap;

typedef struct {
    int src;
    int dest;
    double weight;
} Edge;

typedef struct {
    int V;
    int E;
    Edge edges[MAX_EDGES];
} Graph;

struct Node{
    int val; 
    int prev;
    int used; 
};

typedef struct Node node;

struct table{
    int chosen; // 0: chua chon; 1: danh dau; 2: da chon; -1: chua dong toi
    float val;
    int prev;
};

struct table tab[100][100];
//
struct Belltab{
    int dist;
    int parent;
    int changed;
};

struct Belltab Btab[100][100];
int src, Vert, Ed;
//
struct queueInfo{
    int *queueArr;
    int front;
    int rear;
    int count;
    int capacity;
    bool *onQueue; // Mảng để kiểm tra nhanh một đỉnh có trong queue hay không
};

typedef struct queueInfo *Queue;

int dinh; 
int canh; 
int firstNode, lastNode; 
node x[MAX_VERTICES]; 
int parent[MAX_VERTICES]; 
int num_relax[MAX_VERTICES];
int step = 0;


void init();
void convert_to_matrix(int **a, Graph *graph);
void dijsktra(int **a, int V, int startNode, int endNode, int x_pos, int y_pos); 
void initGraph(Graph* graph, int V);
void addEdge(Graph* graph, int src, int dest, double weight);
bool bellmanFord(Graph* graph, int src, int x_pos, int y_pos, Queue Q);
void BellProcess (struct Belltab Btab[100][100]);
void inputGraphFromKeyboard(Graph* graph);
int readGraphFromFile(Graph* graph, const char* filename);
void saveGraphToFile(Graph* graph, const char* filename);
void pressAnyKeyToContinue();
void chooseAlgorithm();
void urchoice(Graph *graph, int src, int **a, Queue Q); 
void mainMenu();
void freeMatrix(int **a, int rows);
// char print_choice();
void gotoxy(int x, int y);
void drawBox(int x, int y, int width, int height, const char* color);
void printCentered(int boxX, int boxY, int boxWidth, int boxHeight, const char* text, const char* color);
void printProcess();
void initProcess();

//queue
void enqueue (Queue Q, int value);
int dequeue (Queue Q);
bool isEmpty (Queue Q);
bool isFull (Queue Q);
Queue create (unsigned int capacity);
int increase(int value, Queue Q); 

//priority queue
MinHeap* createMinHeap(int capacity);
void swapHeapNodes(HeapNode* a, HeapNode* b);
void minHeapify(MinHeap* minHeap, int idx);
bool isEmptyHeap(MinHeap* minHeap);
HeapNode extractMin(MinHeap* minHeap);
void decreaseKey(MinHeap* minHeap, int vertex, double distance);
void freeMinHeap(MinHeap* minHeap);

int main(){
    system("cls");
    system("chcp 65001 > nul");
    SetConsoleOutputCP(65001);
    Graph graph;
    int choice;
    // char char_choice;
    char filename[100];
    int src = 0;
    int **a = NULL;
    dinh = 0; 
    Queue Q = create(MAX_VERTICES);
    if (Q == NULL) {
        printf("Loi cap phat bo nho cho queue!\n");
        return 1;
    }

    do{
        mainMenu();
        scanf("%d", &choice);

        if (a != NULL && dinh > 0) {
            freeMatrix(a, dinh);
            a = NULL; 
            dinh = 0; 
        }

        switch(choice){
            case 1:
                inputGraphFromKeyboard(&graph);
                printf("\nBạn có muốn lưu đồ thị vào file? (y/n): ");
                char save;
                scanf(" %c", &save);
                if (save == 'y') {
                    printf("Nhập tên file để lưu: ");
                    scanf("%s", filename);
                    saveGraphToFile(&graph, filename);
                }
                dinh = graph.V; 
                a = (int **)malloc(sizeof(int *) * dinh);
                if (a == NULL) {
                    perror("Không thể cấp phát bộ nhớ ");
                    return 1; 
                }
                for (int i = 0; i < dinh; i++){
                    a[i] = (int *)calloc(dinh, sizeof(int) );
                    if (a[i] == NULL) {
                        perror("Không thể cấp phát bộ nhớ ");
                        for(int j = 0; j < i; ++j) free(a[j]);
                        free(a);
                        return 1; 
                    }
                }
                convert_to_matrix(a, &graph); 
                urchoice(&graph, src, a, Q);
                break;
            case 2:
                system("cls");
                gotoxy(65,0);
                printf("Nhập tên file: ");
                scanf("%s", filename);

                if (readGraphFromFile(&graph, filename)) {
                    gotoxy(65,0);
                    printf("Đọc đồ thị từ file thành công!\n");
                    Sleep(1000);
                    system("cls");
                    dinh = graph.V;
                    a = (int **)malloc(sizeof(int *) * dinh);
                     if (a == NULL) {
                        gotoxy(65,0);
                        perror("Không thể cấp phát bộ nhớ ");
                        return 1;
                    }
                    for (int i = 0; i < dinh; i++){
                        a[i] = (int *)calloc(dinh, sizeof(int) );
                         if (a[i] == NULL) {
                            perror("Không thể cấp phát bộ nhớ ");
                            for(int j = 0; j < i; ++j) free(a[j]);
                            free(a);
                            return 1; 
                        }
                    }
                    convert_to_matrix(a, &graph); 
                } else {
                    printf("File chưa có sẵn, có muốn tạo file mới? (y/n)");
                    char create;
                    scanf(" %c", &create);
                    if (create == 'y') {
                        inputGraphFromKeyboard(&graph);
                        saveGraphToFile(&graph, filename);
                        dinh = graph.V; 
                        a = (int **)malloc(sizeof(int *) * dinh);
                         if (a == NULL) {
                            perror("Không thể cấp phát bộ nhớ");
                            return 1;
                        }
                        for (int i = 0; i < dinh; i++){
                            a[i] = (int *)calloc(dinh, sizeof(int) );
                             if (a[i] == NULL) {
                                perror("Không thể cấp phát bộ nhớ");
                                for(int j = 0; j < i; ++j) free(a[j]);
                                free(a);
                                return 1;
                            }
                        }
                        convert_to_matrix(a, &graph);
                    } else {
                        continue; 
                    }
                }
                 urchoice(&graph, src, a, Q); 
                break;
            case 0:
                printf("Thoát chương trình.\n");
                break;
            default:
                gotoxy(63+23+2,13);
                printf("Lựa chọn không hợp lệ. Vui lòng chọn lại.\n");
                Sleep(1000);
                // system("cls");
                // mainMenu();
                // Sleep(1);
        }
    } while(choice != 0);

    if (a != NULL && dinh > 0) {
        freeMatrix(a, dinh);
    }

    return 0;
}

void init(int V, int startNode){ 
    for (int i = 0; i < V; i++){ 
        x[i].val = INT_MAX;
        x[i].prev = -1;
        x[i].used = 0;
    }
    x[startNode].val = 0; 
    x[startNode].prev = -1;
    for(int i = 0; i < dinh; i++){
        if(i != firstNode)
            tab[0][i].chosen = -1;
        else 
            tab[0][i].chosen = 0;
        }
}

void initProcess() {
    for (int i = 0; i < dinh; i++){

        switch(tab[step - 1][i].chosen){
            case 0:
                if(tab[step][i].chosen == 1){
                    break;
                }
                else {
                    tab[step][i].chosen = 0;
                    break;
                }
            case 1:
                tab[step][i].chosen = 2;
                break;
            case 2:
                tab[step][i].chosen = 2;    
                break;
            default: // -1
                tab[step][i].chosen = -1;
                break;
        }
    }
}

void printProcess() {
    // Beginning: 
    printf("\n       ");  // 7 space
    for (int i = 0; i < dinh; i++){
        printf("%-14d", i);
    }
    printf("\n_______"); // 7 space 
    for (int i = 0; i < dinh; i++){
        printf("______________");  // 15 space
    }

    printf("\n ");
    for (int i = 0; i < dinh; i++){
        if (i != firstNode){
            printf("   (INF, -)   ");// 14 space
        }
        else 
            printf("    (0, -)    ");
    } 
    printf("\n ");

    // Main Process:
        // printf("\n%d\n", step);         // CHECK
    for (int i = 2; i < step; i++){
        for (int j = 0; j < dinh; j++){
            
            switch (tab[i][j].chosen){
                case 0: // đỉnh chưa được đánh dấu *
                    if(tab[i][j].val < 10)
                        printf("   (%.1f, %d)   ", tab[i][j].val, tab[i][j].prev);
                    else 
                        printf("  (%.1f, %d)   ", tab[i][j].val, tab[i][j].prev);
                    break;
                case 1: // đỉnh được đánh dấu *
                    printf("       *      ");
                    break;
                case 2: // đỉnh đã được *
                    printf("       -      ");
                    break;
                case -1: // đỉnh chưa động tới
                    printf("   (INF, -)   ");
                    break;
            }

            // printf("(%.2f, %d) ", tab[i][j].val, tab[i][j].prev);    // CHECK
        }
        printf("\n ");
    }
}

void convert_to_matrix(int **a, Graph *graph){
    int V = graph->V;
    int E = graph->E;
    for(int i = 0; i < V; i++) {
        for(int j = 0; j < V; j++) {
            a[i][j] = NO_EDGE;
        }
    }

    for(int i = 0; i < E; i++){
        int s = graph->edges[i].src;
        int e = graph->edges[i].dest;
        int weight = graph->edges[i].weight;
        a[s][e]= weight;
    }
}

// Sửa đổi hàm dijsktra
void dijsktra(int **a, int V, int startNode, int endNode, int x_pos, int y_pos) {
    double dist[MAX_VERTICES];
    int prev[MAX_VERTICES];
    bool visited[MAX_VERTICES]; // Mảng để đánh dấu đỉnh đã được thăm

    MinHeap* minHeap = createMinHeap(V);
    if (minHeap == NULL) {
        gotoxy(x_pos, y_pos);
        printf("Lỗi: Không thể khởi tạo hàng đợi ưu tiên.\n");
        return;
    }

    for (int i = 0; i < V; i++) {
        dist[i] = INT_MAX;
        prev[i] = -1;
        visited[i] = false;
        minHeap->pos[i] = i; // Khởi tạo vị trí ban đầu
    }

    dist[startNode] = 0;
    
    // Thêm tất cả các đỉnh vào min-heap với khoảng cách ban đầu
    for (int i = 0; i < V; ++i) {
        minHeap->nodes[i].vertex = i;
        minHeap->nodes[i].distance = dist[i];
        minHeap->pos[i] = i; // Đảm bảo pos đúng
    }
    minHeap->size = V; // Kích thước ban đầu của heap là V

    // Cập nhật khoảng cách cho startNode trong heap
    decreaseKey(minHeap, startNode, 0);

    // Bảng mô phỏng (tùy chọn)
    for(int i = 0; i < V; i++) {
        if(i != startNode) tab[0][i].chosen = -1;
        else tab[0][i].chosen = 0;
        tab[0][i].val = (i == startNode) ? 0 : INT_MAX;
        tab[0][i].prev = -1;
    }
    step = 1;

    while (!isEmptyHeap(minHeap)) {
        HeapNode extractedNode = extractMin(minHeap);
        int u = extractedNode.vertex;

        if (u == -1) break; // Lỗi khi trích xuất

        if (visited[u]) continue; // Đã thăm đỉnh này, bỏ qua

        visited[u] = true;

        // Cập nhật bảng mô phỏng
        initProcess(step); // Sao chép trạng thái từ bước trước
        tab[step][u].chosen = 1; // Đánh dấu đỉnh đang xét
        
        for (int v = 0; v < V; v++) {
            if (!visited[v] && a[u][v] != NO_EDGE && dist[u] != INT_MAX && (dist[u] + a[u][v] < dist[v])) {
                dist[v] = dist[u] + a[u][v];
                prev[v] = u;
                decreaseKey(minHeap, v, dist[v]);

                // Cập nhật bảng mô phỏng
                tab[step][v].val = dist[v];
                tab[step][v].prev = prev[v];
                tab[step][v].chosen = 0; // Đỉnh được cập nhật nhưng chưa được chọn
            } else if (!visited[v] && tab[step][v].chosen == -1) { // Đỉnh chưa được thăm và chưa được đụng đến
                 tab[step][v].val = INT_MAX;
                 tab[step][v].prev = -1;
                 tab[step][v].chosen = -1;
            }
        }
        tab[step][u].chosen = 2; // Đánh dấu đỉnh đã được xử lý (đã chọn)
        step++;
    }

    if (dist[endNode] == INT_MAX) {
        gotoxy(x_pos, y_pos);
        printf("Không có đường đi từ điểm %d đến điểm %d\n", startNode, endNode);
    } else {
        gotoxy(x_pos, y_pos);
        printf("\nĐường đi ngắn nhất từ điểm %d đến điểm %d:\n", startNode, endNode);
        int path[MAX_VERTICES];
        int count = 0;
        int currentNode = endNode;

        while (currentNode != -1) {
            path[count++] = currentNode;
            currentNode = prev[currentNode];
        }
        gotoxy(x_pos, y_pos + 3);
        for (int j = count - 1; j >= 0; j--) {
            printf("%d", path[j]);
            if (j > 0) {
                printf(" -> ");
            }
        }
        printf("\n");
        gotoxy(x_pos, y_pos + 4);
        printf("Độ dài đường đi ngắn nhất từ điểm %d đến điểm %d là: %.0f\n", startNode, endNode, dist[endNode]);
        // printf("Bạn có muốn in steps không? (y/n): ");
        // char print_choice;
        // scanf(" %c", &print_choice);
        // if (print_choice == 'y') {
        //     printProcess(step);
        // }
    }
    freeMinHeap(minHeap); // Giải phóng bộ nhớ cho minHeap
}

void initGraph(Graph* graph, int V) {
    graph->V = V;
    graph->E = 0;
}

void addEdge(Graph* graph, int src, int dest, double weight) {
    if (graph->E < MAX_EDGES) {
        graph->edges[graph->E].src = src;
        graph->edges[graph->E].dest = dest;
        graph->edges[graph->E].weight = weight;
        graph->E++;
    } else {
        printf("Cảnh báo: Đã đạt tối đa đỉnh thêm vào. Vui lòng không thêm nữa.\n");
    }
}

bool bellmanFord(Graph* graph, int src, int x_pos, int y_pos, Queue Q) {
    int V = graph->V;
    int E = graph->E;
    int dist[MAX_VERTICES];

    for (int i = 0; i < V; i++) {
        dist[i] = INT_MAX;
        parent[i] = -1;
        num_relax[i] = 0;
        Q->onQueue[i] = false;
        Btab[0][i].dist = INT_MAX;
        Btab[0][i].parent = -1;
    }
    
    Btab[0][src].dist = 0;
    Btab[0][src].parent = 0;
    dist[src] = 0;
    enqueue(Q, src);
    Q->onQueue[src] = true;

    while (!isEmpty(Q)) {
        int u = dequeue(Q);
        Q->onQueue[u] = false; 

        // for (int j = 0; j < V; j++){
        //     Btab[i][j].changed = 0;
        // }

        for (int j = 0; j < E; j++) {
            if (graph->edges[j].src == u) { 
                int v = graph->edges[j].dest;
                int weight = graph->edges[j].weight;

                if (dist[u] != INT_MAX && dist[u] + weight < dist[v]) {
                    dist[v] = dist[u] + weight;
                    parent[v] = u;
                    num_relax[v]++;
                    // Btab[i][v].dist = dist[v];
                    // Btab[i][v].parent = parent[v];
                    // Btab[i][v].changed = 1;

                    if (num_relax[v] >= V) {
                        gotoxy(x_pos,y_pos);
                        printf("Đồ thị chứa chu trình âm!\n");
                        return false;
                    }

                    if (!Q->onQueue[v]) { 
                        enqueue(Q, v);
                        Q->onQueue[v] = true;
                    }
                }
            }
        }
        // for(int j = 0; j < V; j++){
        //     if(Btab[i][j].changed == 0){
        //         Btab[i][j].dist = Btab[i-1][j].dist;
        //         Btab[i][j].parent = Btab[i-1][j].parent;
        //     }
        // }
    }

    gotoxy(x_pos,y_pos+1);
    printf("\nKhoảng cách và đường đi từ đỉnh %d:\n", src);
    for (int i = 0; i < V; i++) {
        if (dist[i] == INT_MAX) {
            gotoxy(x_pos,y_pos+2+i);
            printf("Đỉnh %d: Không thể đến được\n", i);
        } else {
            gotoxy(x_pos,y_pos+2+i);
            printf("Đỉnh %d: Khoảng cách = %d, Đường đi: ", i, dist[i]);
            int path[MAX_VERTICES];
            int count = 0;
            int current = i;
        
            while (current != -1) {
                path[count++] = current;
                current = parent[current];
            }

            for (int j = count - 1; j >= 0; j--) {
                printf("%d", path[j]);
                if (j > 0) printf(" -> ");
            }
            printf("\n");
        }
    }

    // BellProcess(Btab);

    return true;
}

void BellProcess (struct Belltab Btab[100][100]) {
    // Beginning: 
    printf("\nBang qua trinh tim duong di (Voi k la so lan lap):\n");
    printf("\nk        ");  // 9 space
    for (int i = 0; i < Vert; i++){
        printf("%-14d", i);
    }
    printf("\n________"); // 8 space 
    for (int i = 0; i < Vert; i++){
        printf("______________");  // 15 space
    }

    printf("\n0  ");  // 3 space
    for (int i = 0; i < Vert; i++){
        if (i != src){
            printf("   (INF, -)   ");// 14 space
        }
        else 
            printf("    (0, -)    ");
    } 
    printf("\n");

    // Main Process:
    for (int i = 1; i < Vert; i++){
        if (i < 10)                                                 //if (i < 10 && i != 1)//
            printf("%d  ", i); // 3 space
        // else if ( i == 3)                                        //CHECK//
        //     printf("%d ", 11);
        else
            printf("%d ", i); // 3 space
        
        // Btab[1][3].dist = 11;                                             //CHECK//
        // Btab[1][0].dist = 11;
        // Btab[2][3].dist = -1;
        // Btab[2][0].dist = -1;
        // Btab[3][0].dist = -11;
        // Btab[3][3].dist = -11;

        if (Btab[i][0].dist < 10 && Btab[i][0].dist >= 0)
            printf("   (%d.0, %d)   ", Btab[i][0].dist, Btab[i][0].parent); // 1.0 
        else if (abs(Btab[i][0].dist) > 10 && Btab[i][0].dist < 0)
            printf(" (%d.0, %d)   ", Btab[i][0].dist, Btab[i][0].parent);  // -11.0
        else   
            printf("  (%d.0, %d)   ", Btab[i][0].dist, Btab[i][0].parent);  // 11.0 & -1.0
            
        for (int j = 1; j < Vert; j++){
            if (Btab[i][j].dist < 10 && Btab[i][j].dist > 0) 
                printf("   (%d.0, %d)   ", Btab[i][j].dist, Btab[i][j].parent); // 1.0
            else if (abs(Btab[i][j].dist) > 10 && Btab[i][j].dist < 0)
                printf(" (%d.0, %d)   ", Btab[i][j].dist, Btab[i][j].parent);  // -11.0
            else 
                printf("  (%d.0, %d)   ", Btab[i][j].dist, Btab[i][j].parent);  // 11.0 & -1.0
            }
        printf("\n");
    }

    if (Vert < 10)
            printf("%d  ", Vert); // 3 space
        else
            printf("%d ", Vert); // 3 space
            
    // Btab[Vert - 1][0].dist = 11;                                             //CHECK//
    // Btab[Vert - 1][4].dist = 11;
    // Btab[Vert - 1][0].dist = -1;
    // Btab[Vert - 1][4].dist = -1;
    // Btab[Vert - 1][0].dist = -11;
    // Btab[Vert - 1][4].dist = -11;


    if(Btab[Vert - 1][0].dist < 10 && Btab[Vert - 1][0].dist >= 0)
        printf("   (%d.0, %d)   ", Btab[Vert - 1][0].dist, Btab[Vert - 1][0].parent);  // 1.0
    else if (abs(Btab[Vert - 1][0].dist) > 10 && Btab[Vert - 1][0].dist < 0)
            printf(" (%d.0, %d)   ", Btab[Vert - 1][0].dist, Btab[Vert - 1][0].parent);  // -11.0
    else 
        printf("  (%d.0, %d)   ", Btab[Vert - 1][0].dist, Btab[Vert - 1][0].parent);  // -1.0 & -11.0
            
    for (int j = 1; j < Vert; j++){
        if(Btab[Vert - 1][j].dist < 10 && Btab[Vert - 1][j].dist > 0)
            printf("   (%d.0, %d)   ", Btab[Vert - 1][j].dist, Btab[Vert - 1][j].parent);  // 1.0 
        else if (abs(Btab[Vert - 1][j].dist) > 10 && Btab[Vert - 1][j].dist < 0)
            printf(" (%d.0, %d)   ", Btab[Vert - 1][j].dist, Btab[Vert - 1][j].parent);  // -11.0
        else 
            printf("  (%d.0, %d)   ", Btab[Vert - 1][j].dist, Btab[Vert - 1][j].parent);  // -1.0 & -11.0
    }
    printf("\n");

}

void inputGraphFromKeyboard(Graph* graph) {
    int graphType;
    system("cls");
    gotoxy(65,0);
    printf("%sNhập kiểu đồ thị: %sNếu vô hướng nhập 0, có hướng nhập 1 (0/1)? ",YELLOW, WHITE);
    scanf("%d",&graphType);
    int V, E;
    system("cls");
    gotoxy(65,1);
    printf("%sNhập số đỉnh: ",YELLOW);
    gotoxy(65,2);
    printf("%sNhập số cạnh: ",YELLOW);
    gotoxy(65,4);
    printf("%sChú ý: %sSố đỉnh và số cạnh không được vượt quá %d và %d\n",RED,WHITE, MAX_VERTICES, MAX_EDGES);
    gotoxy(65+14,1);
    scanf("%d", &V);
    while (V>MAX_VERTICES){
        gotoxy(65+14,1);
        printf("Lỗi: Số đỉnh vượt quá giới hạn cho phép (%d đỉnh).\n", MAX_VERTICES);
        Sleep(2000);
        gotoxy(65+14,1);
        printf("                                                                  ");
        gotoxy(65+14,1);
        scanf("%d", &V);
        initGraph(graph, 0);
    }
    gotoxy(65+14,2);
    scanf("%d", &E);

    if (E > MAX_EDGES) {
        gotoxy(65+14,2);
        printf("Lỗi: Số cạnh vượt quá giới hạn cho phép (%d cạnh).\n", MAX_EDGES);
        Sleep(2000);
        gotoxy(65+14,2);
        printf("                                                                  ");
        gotoxy(65+14,2);
        scanf("%d", &E);
        initGraph(graph, 0);
    }

    initGraph(graph, V);
    system("cls");
    gotoxy(65,1);
    printf("Nhập danh sách cạnh (đỉnh đầu, đỉnh cuối, khoảng cách):\n");
    for (int i = 0; i < E; i++) {
        gotoxy(65,i+2);
        int src, dest;
        double weight;
        scanf("%d %d %lf", &src, &dest, &weight);
        if (src >= 0 && src < V && dest < V) {
            addEdge(graph, src, dest, weight);
            if (graphType==0){
                addEdge(graph, dest, src, weight);
            }
        } else {
            gotoxy(65,i+2);
            printf("Cảnh báo: Chỉ số không hợp lệ (%d, %d) cho cạnh %d. Bỏ qua...\n", src, dest, i);
            Sleep(1000);
            gotoxy(65,i+2-1);
            printf("                                                                    ");
            gotoxy(65,i+2-1);
            graph->E--;
        }
    }
    system("cls");
    gotoxy(65,0);
    printf("Đây có phải đồ thị bạn đã nhập không?");
    gotoxy(65,1);
    if (graphType == 0) {
        printf("Đồ thị vô hướng\n");
    } else {
        printf("Đồ thị có hướng\n");
    }
    gotoxy(65,2);
    printf("Số đỉnh: %d\n", graph->V);
    gotoxy(65,3);
    printf("Số cạnh: %d\n", graph->E/2);
    gotoxy(65,4);
    printf("Danh sách cạnh:\n");
    if (graphType == 0){
        for (int i = 0; i < graph->E; i+=2) {
            gotoxy(65,i/2+5);
            printf("%d %d %.lf\n", graph->edges[i].src, graph->edges[i].dest, graph->edges[i].weight);
        }
        gotoxy(65,graph->E/2+5);
    }
    else if (graphType == 1){
        for (int i = 0; i< graph->E; i++){
            gotoxy(65,i+5);
            printf("%d %d %.lf\n", graph->edges[i].src, graph->edges[i].dest, graph->edges[i].weight);
        }
        gotoxy(65,graph->E+5);
    }
    
    printf("(y/n)? ");
    char confirm;
    scanf(" %c", &confirm);
    if (confirm == 'n') {
        system("cls");
        gotoxy(65,0);
        printf("Nhập lại đồ thị từ bàn phím.\n");
        Sleep(1000);
        inputGraphFromKeyboard(graph);
    } else {
        system("cls");
    }
}

int readGraphFromFile(Graph* graph, const char* filename) {
    system("cls");
    FILE* file = fopen(filename, "r");
    if (file == NULL) {
        return 0;
    }

    int V, E;
    if (fscanf(file, "%d %d", &V, &E) != 2) {
        gotoxy(65,0);
        printf("Lỗi đọc file không thành công.\n");
        fclose(file);
        return 0;
    }

    if (V > MAX_VERTICES || E > MAX_EDGES) {
        printf("Lỗi: Đồ thị trong file vượt mức cho phép (%d đỉnh, %d cạnh).\n", MAX_VERTICES, MAX_EDGES);
        fclose(file);
        return 0;
    }

    initGraph(graph, V);

    for (int i = 0; i < E; i++) {
        int src, dest, weight;
        if (fscanf(file, "%d %d %d", &src, &dest, &weight) != 3) {
             printf("Lỗi không thể đọc cạnh %d từ file.\n", i);
             continue; 
        }
        if (src >= 0 && src < V && dest >= 0 && dest < V) {
             addEdge(graph, src, dest, weight);
        } else {
             printf("Cảnh báo: Chỉ số không hợp lệ (%d, %d) cho cạnh %d trong file. Bỏ qua...\n", src, dest, i);
             graph->E--;
        }
    }

    fclose(file);
    return 1;
}

void saveGraphToFile(Graph* graph, const char* filename) {
    FILE* file = fopen(filename, "w");
    if (file == NULL) {
        printf("Không thể mở file để ghi!\n");
        return;
    }

    fprintf(file, "%d %d\n", graph->V, graph->E);
    for (int i = 0; i < graph->E; i++) {
        fprintf(file, "%d %d %lf\n",
                graph->edges[i].src,
                graph->edges[i].dest,
                graph->edges[i].weight);
    }

    fclose(file);
    printf("Đã lưu đồ thị vào file %s\n", filename);
}

void pressAnyKeyToContinue() {
    system("chcp 65001 > nul");
    SetConsoleOutputCP(65001);  
    printf("\nNhấn phím bất kỳ để tiếp tục...");
    while (getchar() != '\n');
    getchar();
}

void chooseAlgorithm() { 
    system("cls");
    drawBox(65, 0, 60, 7, CYAN);
    printCentered(67, 0, 58, 2, "----- Chọn thuật toán -----", MINT_GREEN);
    printCentered(63, 1, 58, 2, "1. Dijkstra", WHITE);
    printCentered(65, 2, 58, 2, "2. Bellman-Ford", WHITE);
    printCentered(63+8, 3, 58, 2, "3. So sánh 2 thuật toán", WHITE);
    printCentered(63+7, 4, 58, 2, "4. Trở về menu chính", WHITE);
    gotoxy(65, 8);
    printf("Lựa chọn của bạn: ");
}

void urchoice(Graph *graph, int src, int **a, Queue Q) {
    int algo_choice; 
    clock_t start, end;
    double time_taken_Dijkstra, time_taken_BF;
    int step=0;
    system("chcp 65001 > nul");
    SetConsoleOutputCP(65001);  
    if (graph->V <= 0) {
        printf("Không có đồ thị để thực hiện thuật toán.\n");
        return;
    }

    do {
        chooseAlgorithm();
        scanf("%d", &algo_choice); 
        switch(algo_choice) {
            case 1:
                system("cls");
                printf("Nhập điểm bắt đầu và điểm kết thúc (0-%d): ", graph->V - 1);
                scanf("%d %d", &firstNode, &lastNode); 

                if (firstNode < 0 || firstNode >= graph->V || lastNode < 0 || lastNode >= graph->V) {
                    printf("Lỗi: Đỉnh bắt đầu hoặc kết thúc không hợp lệ.\n");
                    break; 
                }
                start = clock();
                dijsktra(a, graph->V, firstNode, lastNode,0 ,1); 
                end = clock();
                printf("\nThời gian thực hiện thuật toán: %.5f giây\n", (double)(end - start) / CLOCKS_PER_SEC);
                printf("Bạn có muốn in steps không? (y/n): ");
                char print_choice;
                scanf(" %c", &print_choice);
                if (print_choice == 'y') {
                printProcess(step);
                }
                break;
            case 2:
                system("cls");
                printf("\nNhập điểm bắt đầu (0-%d): ", graph->V-1);
                scanf("%d", &src);

                if (src < 0 || src >= graph->V) {
                     printf("Lỗi: Điểm bắt đầu không hợp lệ.\n");
                     break; 
                }
                start = clock();
                bellmanFord(graph, src,0,1, Q);               
                end = clock();
                printf("\nThời gian thực hiện thuật toán: %.5f giây\n", (double)(end - start) / CLOCKS_PER_SEC);
                break;
            case 3:
                system("cls");
                gotoxy(65, 0);
                printf("So sánh thời gian thực hiện giữa Dijkstra và Bellman-Ford:\n");
                gotoxy(55, 1);
                printf("Thuật toán Dijkstra sẽ chạy từ đỉnh 0 đến đỉnh %d\n", graph->V-1);
                gotoxy(55, 2);
                printf("Thuật toán Bellman-Ford sẽ chạy từ đỉnh 0 đến tất cả các đỉnh còn lại.\n");
                gotoxy(0, 3);
                start = clock();
                dijsktra(a, graph->V, 0, graph->V-1,0,3); 
                end = clock();
                time_taken_Dijkstra = (double)(end - start) / CLOCKS_PER_SEC;
                
   
                start = clock();
                bellmanFord(graph, 0,110,3,Q); 
                end = clock();
                time_taken_BF = (double)(end - start) / CLOCKS_PER_SEC;
                gotoxy(65, graph->E/2+5+3);    
                printf("\nThời gian thực hiện thuật toán Dijkstra nhanh hơn Bellman-Ford %.5lf giây.\n",time_taken_BF-time_taken_Dijkstra);
                break;
            case 4:
                printf("Trở về menu chính.\n");
                break;
            default:
                printf("Lựa chọn không hợp lệ. Vui lòng chọn lại.\n");
        }
        if (algo_choice != 4) {
             pressAnyKeyToContinue();
             system("cls");
        }
    } while (algo_choice != 4);
}

void mainMenu() {
    system("cls"); 

    drawBox(65, 0, 60, 8, CYAN);

    system ("chcp 65001 > nul");
    printCentered(67, 0, 58, 2, "PBL1: The shortest path algorithm", MINT_GREEN);
    printCentered(67, 1, 58, 2, "Dijsktra & Bellman-Ford", MINT_GREEN);
    printCentered(57, 2, 36, 4, "Tên SV: ", MINT_BLUE);
    printCentered(67, 2, 36, 4, "Phan Vũ Long", WHITE);
    printCentered(75, 3, 36, 5, "Trương Thị Ngọc Huyền", WHITE);
    printCentered(85, 2, 36, 4, "GVHD: ", MINT_BLUE);
    printCentered(98, 2, 36, 4, "Nguyễn Văn Hiệu", WHITE);

    gotoxy(65, 10);
    printf("1. Nhập đồ thị từ bàn phím\n");
    gotoxy(65, 11);
    printf("2. Đọc đồ thị từ file\n");
    gotoxy(65, 12);
    printf("0. Thoát\n");
    gotoxy(65, 13);
    printf("Nhập lựa chọn tại đây: ");
}

void freeMatrix(int **a, int rows){
    if (a==NULL) return;
    for (int i = 0; i < rows; i++){
        free(a[i]);
    }
    free(a);
}

void gotoxy(int x, int y) {
    COORD coord;
    coord.X = x;
    coord.Y = y;
    SetConsoleCursorPosition(GetStdHandle(STD_OUTPUT_HANDLE), coord);
}

void drawBox(int x, int y, int width, int height, const char* color) {
    system ("chcp 437 >nul");
    int i, j;

    gotoxy(x, y);
    printf("%s%c",color, 201);

    gotoxy(x + width, y);
    printf("%s%c",color, 187);

    gotoxy(x, y + height);
    printf("%s%c",color, 200);

    gotoxy(x + width, y + height);
    printf("%s%c",color, 188);

    for (i = 1; i < width; i++) {
        gotoxy(x + i, y);
        printf("%s%c",color, 205);
        gotoxy(x + i, y + height);
        printf("%s%c",color, 205);
    }

    for (j = 1; j < height; j++) {
        gotoxy(x, y + j);
        printf("%s%c",color, 186);
        gotoxy(x + width, y + j);
        printf("%s%c",color, 186);
    }

    system ("chcp 65001 > nul");
}

void printCentered(int boxX, int boxY, int boxWidth, int boxHeight, const char* text, const char* color) {
    int textLength = strlen(text);
    int centerX = boxX + (boxWidth - textLength) / 2;
    int centerY = boxY + boxHeight / 2;
    
    gotoxy(centerX, centerY);
    printf("%s%s", color, text);
}

void enqueue (Queue Q, int value){
    if (isFull(Q)){
        printf("Hang doi da day!\n");
        return;
    }
    Q->rear = increase(Q->rear, Q);
    Q->count++;
    Q->queueArr[Q->rear] = value;
}

int dequeue (Queue Q){
    if (isEmpty(Q)){
        return -1; 
    }
    int value = Q->queueArr[Q->front];
    Q->front = increase(Q->front, Q);
    Q->count--;
    return value;
}

bool isEmpty (Queue Q){
    return Q->count == 0;
}

bool isFull (Queue Q){
    return Q->count == Q->capacity;
}

int increase(int value, Queue Q){
    return (value + 1) % Q->capacity;
}

Queue create (unsigned int capacity){
    Queue Q = malloc(sizeof(struct queueInfo));
    if (Q==NULL) return NULL;
    Q->queueArr = malloc(capacity * sizeof(int));
    if (Q->queueArr == NULL) {
        free(Q);
        return NULL;
    }
    Q->onQueue = calloc(capacity, sizeof(bool));
    if (Q->onQueue == NULL) {
        free(Q->queueArr);
        free(Q);
        return NULL;
    }
    Q->capacity = capacity;
    Q->count = 0;
    Q->front = 0;
    Q->rear = -1;
    return Q;
}

MinHeap* createMinHeap(int capacity) {
    MinHeap* minHeap = (MinHeap*) malloc(sizeof(MinHeap));
    if (minHeap == NULL) {
        perror("Lỗi cấp phát bộ nhớ cho MinHeap");
        return NULL;
    }
    minHeap->pos = (int*) malloc(capacity * sizeof(int));
    if (minHeap->pos == NULL) {
        perror("Lỗi cấp phát bộ nhớ cho MinHeap->pos");
        free(minHeap);
        return NULL;
    }
    minHeap->nodes = (HeapNode*) malloc(capacity * sizeof(HeapNode));
    if (minHeap->nodes == NULL) {
        perror("Lỗi cấp phát bộ nhớ cho MinHeap->nodes");
        free(minHeap->pos);
        free(minHeap);
        return NULL;
    }
    minHeap->size = 0;
    minHeap->capacity = capacity;
    return minHeap;
}

void swapHeapNodes(HeapNode* a, HeapNode* b) {
    HeapNode temp = *a;
    *a = *b;
    *b = temp;
}

void minHeapify(MinHeap* minHeap, int idx) {
    int smallest = idx;
    int left = 2 * idx + 1;
    int right = 2 * idx + 2;

    if (left < minHeap->size && minHeap->nodes[left].distance < minHeap->nodes[smallest].distance) {
        smallest = left;
    }

    if (right < minHeap->size && minHeap->nodes[right].distance < minHeap->nodes[smallest].distance) {
        smallest = right;
    }

    if (smallest != idx) {
        minHeap->pos[minHeap->nodes[smallest].vertex] = idx;
        minHeap->pos[minHeap->nodes[idx].vertex] = smallest;

        swapHeapNodes(&minHeap->nodes[smallest], &minHeap->nodes[idx]);
        minHeapify(minHeap, smallest);
    }
}

bool isEmptyHeap(MinHeap* minHeap) {
    return minHeap->size == 0;
}

HeapNode extractMin(MinHeap* minHeap) {
    if (isEmptyHeap(minHeap)) {
        HeapNode emptyNode = {-1, -1.0}; 
        return emptyNode;
    }

    HeapNode root = minHeap->nodes[0];
    HeapNode lastNode = minHeap->nodes[minHeap->size - 1];
    minHeap->nodes[0] = lastNode;

    minHeap->pos[root.vertex] = minHeap->size - 1;
    minHeap->pos[lastNode.vertex] = 0;

    minHeap->size--;
    minHeapify(minHeap, 0);

    return root;
}

void decreaseKey(MinHeap* minHeap, int vertex, double distance) {
    int i = minHeap->pos[vertex];
    minHeap->nodes[i].distance = distance;

    while (i && minHeap->nodes[i].distance < minHeap->nodes[(i - 1) / 2].distance) {
        minHeap->pos[minHeap->nodes[i].vertex] = (i - 1) / 2;
        minHeap->pos[minHeap->nodes[(i - 1) / 2].vertex] = i;
        swapHeapNodes(&minHeap->nodes[i], &minHeap->nodes[(i - 1) / 2]);
        i = (i - 1) / 2;
    }
}

void freeMinHeap(MinHeap* minHeap) {
    if (minHeap) {
        free(minHeap->pos);
        free(minHeap->nodes);
        free(minHeap);
    }
}

// them ham in ra bang mo phong cho bellmanford
// trong phan so sanh thuat toan, cho ng dung chon so sanh thoi gian chay 
