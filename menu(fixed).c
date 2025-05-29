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
#define MAX_EDGES 4950
#define NO_EDGE -1.0
#define HEAP_MAX_SIZE MAX_VERTICES
#define QUEUE_SIZE MAX_EDGES

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

typedef struct {
    int vertex;
    double distance;
} HeapNode;

typedef struct {
    HeapNode* nodes;
    int size;
    int capacity;
    int* pos;
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

typedef struct {
    double val;
    int prev;
    bool selectedInStep;
} DijkstraProcessEntry;

DijkstraProcessEntry dijkstraProcessTab[MAX_VERTICES + 1][MAX_VERTICES];

struct Belltab {
    double dist;
    int parent;
    bool changed;
};

struct Belltab Btab[MAX_VERTICES][MAX_VERTICES];
int src_bellman_global, Vert_bellman_global, Ed_bellman_global;

double **adjMatrix_dijkstra_global = NULL;

typedef struct {
    Edge data[QUEUE_SIZE];
    int front;
    int rear;
    int count;
} CircularQueue;

void initGraph(Graph* graph, int V);
void addEdge(Graph* graph, int src, int dest, double weight);
void inputGraphFromKeyboard(Graph* graph);
int readGraphFromFile(Graph* graph, const char* filename);
void saveGraphToFile(Graph* graph, const char* filename);
void freeMatrix(double **a, int rows);

void gotoxy(int x, int y);
void drawBox(int x, int y, int width, int height, const char* color);
void printCentered(int boxX, int boxY, int boxWidth, int boxHeight, const char* text, const char* color);
void pressAnyKeyToContinue();
void mainMenu();
void urchoice(Graph *graph);
void chooseAlgorithm();

void convert_to_matrix(double **adjMatrix, Graph *graph);
void dijkstra(double **adjMatrix, int V, int startNode, int endNode);
void printDijkstraProcess(int V, int startNode, int totalSteps);

bool bellmanFord(Graph* graph, int src);
void printBellmanProcess(int V, int startNode);
void printBellmanResult(double dist[MAX_VERTICES], int parent[MAX_VERTICES], int V, int src);

void initQueue(CircularQueue* q);
bool isQueueEmpty(CircularQueue* q);
bool isQueueFull(CircularQueue* q);
void enqueue(CircularQueue* q, Edge item);
Edge dequeue(CircularQueue* q);

MinHeap* createMinHeap(int capacity);
void swapHeapNodes(HeapNode* a, HeapNode* b);
void minHeapify(MinHeap* minHeap, int idx);
bool isEmptyHeap(MinHeap* minHeap);
HeapNode extractMin(MinHeap* minHeap);
void decreaseKey(MinHeap* minHeap, int vertex, double distance);
void freeMinHeap(MinHeap* minHeap);
void insertHeap(MinHeap* minHeap, double distance, int vertex);

int main(){
    system("cls");
    system("chcp 65001 > nul");
    SetConsoleOutputCP(65001);
    Graph graph;
    int choice;
    char filename[100];

    do{
        mainMenu();
        scanf("%d", &choice);
        while ((getchar()) != '\n');

        if (adjMatrix_dijkstra_global != NULL) {
            freeMatrix(adjMatrix_dijkstra_global, graph.V);
            adjMatrix_dijkstra_global = NULL;
        }

        switch(choice){
            case 1:
                inputGraphFromKeyboard(&graph);
                printf("\nBạn có muốn lưu đồ thị vào file? (y/n): ");
                char save;
                scanf(" %c", &save);
                while ((getchar()) != '\n');
                if (save == 'y') {
                    printf("Nhập tên file để lưu: ");
                    scanf("%s", filename);
                    while ((getchar()) != '\n');
                    saveGraphToFile(&graph, filename);
                }

                adjMatrix_dijkstra_global = (double **)malloc(sizeof(double *) * graph.V);
                if (adjMatrix_dijkstra_global == NULL) {
                    perror("Không thể cấp phát bộ nhớ cho adjMatrix_dijkstra_global");
                    return 1;
                }
                for (int i = 0; i < graph.V; i++){
                    adjMatrix_dijkstra_global[i] = (double *)calloc(graph.V, sizeof(double));
                    if (adjMatrix_dijkstra_global[i] == NULL) {
                        perror("Không thể cấp phát bộ nhớ cho hàng của adjMatrix_dijkstra_global");
                        freeMatrix(adjMatrix_dijkstra_global, i);
                        adjMatrix_dijkstra_global = NULL;
                        return 1;
                    }
                }
                convert_to_matrix(adjMatrix_dijkstra_global, &graph);
                urchoice(&graph);
                break;
            case 2:
                system("cls");
                gotoxy(65,0);
                printf("Nhập tên file: ");
                scanf("%s", filename);
                while ((getchar()) != '\n');

                if (readGraphFromFile(&graph, filename)) {
                    gotoxy(65,0);
                    printf("Đọc đồ thị từ file thành công!\n");
                    Sleep(1000);
                    system("cls");

                    adjMatrix_dijkstra_global = (double **)malloc(sizeof(double *) * graph.V);
                    if (adjMatrix_dijkstra_global == NULL) {
                        perror("Không thể cấp phát bộ nhớ cho adjMatrix_dijkstra_global");
                        return 1;
                    }
                    for (int i = 0; i < graph.V; i++){
                        adjMatrix_dijkstra_global[i] = (double *)calloc(graph.V, sizeof(double));
                        if (adjMatrix_dijkstra_global[i] == NULL) {
                            perror("Không thể cấp phát bộ nhớ cho hàng của adjMatrix_dijkstra_global");
                            freeMatrix(adjMatrix_dijkstra_global, i);
                            adjMatrix_dijkstra_global = NULL;
                            return 1;
                        }
                    }
                    convert_to_matrix(adjMatrix_dijkstra_global, &graph);
                } else {
                    printf("File chưa có sẵn, có muốn tạo file mới? (y/n)");
                    char create;
                    scanf(" %c", &create);
                    while ((getchar()) != '\n');
                    if (create == 'y') {
                        inputGraphFromKeyboard(&graph);
                        saveGraphToFile(&graph, filename);

                        adjMatrix_dijkstra_global = (double **)malloc(sizeof(double *) * graph.V);
                        if (adjMatrix_dijkstra_global == NULL) {
                            perror("Không thể cấp phát bộ nhớ cho adjMatrix_dijkstra_global");
                            return 1;
                        }
                        for (int i = 0; i < graph.V; i++){
                            adjMatrix_dijkstra_global[i] = (double *)calloc(graph.V, sizeof(double));
                            if (adjMatrix_dijkstra_global[i] == NULL) {
                                perror("Không thể cấp phát bộ nhớ cho hàng của adjMatrix_dijkstra_global");
                                freeMatrix(adjMatrix_dijkstra_global, i);
                                adjMatrix_dijkstra_global = NULL;
                                return 1;
                            }
                        }
                        convert_to_matrix(adjMatrix_dijkstra_global, &graph);
                    } else {
                        continue;
                    }
                }
                urchoice(&graph);
                break;
            case 0:
                printf("Thoát chương trình.\n");
                break;
            default:
                gotoxy(63+23+2,13);
                printf("Lựa chọn không hợp lệ. Vui lòng chọn lại.\n");
                Sleep(1000);
        }
    } while(choice != 0);

    if (adjMatrix_dijkstra_global != NULL) {
        freeMatrix(adjMatrix_dijkstra_global, graph.V);
    }

    return 0;
}

void printDijkstraProcess(int V, int startNode, int totalSteps) {
    printf("\nBảng quá trình tìm đường đi (Với k là số lần lặp):\n");
    printf("\nk        ");
    for (int i = 0; i < V; i++) {
        printf("%14d", i);
    }
    printf("\n_________");
    for (int i = 0; i < V; i++) {
        printf("______________");
    }
    printf("\n");

    printf("0        ");
    for (int i = 0; i < V; i++) {
        if (dijkstraProcessTab[0][i].val == INFINITY) {
            printf("   (INF, -)   ");
        } else {
            printf("  (%1.0lf, -)   ", dijkstraProcessTab[0][i].val);
        }
    }
    printf("\n");

    for (int k = 1; k <= totalSteps; k++) {
        printf("%-8d ", k);
        for (int j = 0; j < V; j++) {
            if (dijkstraProcessTab[k][j].val == INFINITY) {
                printf(" (INF, -)   ");
            } else {
                if (dijkstraProcessTab[k][j].val < 10 && dijkstraProcessTab[k][j].val >= 0) {
                    printf("(%.1lf, %2d)   ", dijkstraProcessTab[k][j].val, dijkstraProcessTab[k][j].prev);
                } else if (dijkstraProcessTab[k][j].val < 100 && dijkstraProcessTab[k][j].val >= 0) {
                    printf("(%.1lf, %2d)   ", dijkstraProcessTab[k][j].val, dijkstraProcessTab[k][j].prev);
                } else if (dijkstraProcessTab[k][j].val >= 100 && dijkstraProcessTab[k][j].val < INFINITY) {
                    printf("(%.1lf, %2d)  ", dijkstraProcessTab[k][j].val, dijkstraProcessTab[k][j].prev);
                }
                else {
                    printf("(%.1lf, %2d)  ", dijkstraProcessTab[k][j].val, dijkstraProcessTab[k][j].prev);
                }
            }
        }
        printf("\n");
    }
}

void convert_to_matrix(double **adjMatrix, Graph *graph){
    int V = graph->V;
    int E = graph->E;
    for(int i = 0; i < V; i++) {
        for(int j = 0; j < V; j++) {
            adjMatrix[i][j] = NO_EDGE;
        }
    }

    for(int i = 0; i < E; i++){
        int s = graph->edges[i].src;
        int e = graph->edges[i].dest;
        double weight = graph->edges[i].weight;
        adjMatrix[s][e]= weight;
    }
}

void dijkstra(double **adjMatrix, int V, int startNode, int endNode) {
    double dist[MAX_VERTICES];
    int prev[MAX_VERTICES];
    bool visited[MAX_VERTICES];
    int step_counter = 0;

    MinHeap* minHeap = createMinHeap(V);
    if (minHeap == NULL) {
        printf("Lỗi: Không thể khởi tạo hàng đợi ưu tiên.\n");
        return;
    }

    for (int i = 0; i < V; i++) {
        dist[i] = INFINITY;
        prev[i] = -1;
        visited[i] = false;
    }

    dist[startNode] = 0;
    insertHeap(minHeap, 0, startNode);

    for (int i = 0; i < V; i++) {
        dijkstraProcessTab[0][i].val = (i == startNode) ? 0.0 : INFINITY;
        dijkstraProcessTab[0][i].prev = -1;
        dijkstraProcessTab[0][i].selectedInStep = false;
    }

    while (!isEmptyHeap(minHeap)) {
        HeapNode extractedNode = extractMin(minHeap);
        int u = extractedNode.vertex;
        double current_dist_u = extractedNode.distance;

        if (visited[u]) {
            continue;
        }
        
        step_counter++;
        
        for (int i = 0; i < V; i++) {
            dijkstraProcessTab[step_counter][i].val = dist[i];
            dijkstraProcessTab[step_counter][i].prev = prev[i];
            dijkstraProcessTab[step_counter][i].selectedInStep = false;
        }

        visited[u] = true;
        dijkstraProcessTab[step_counter][u].selectedInStep = true;

        for (int v = 0; v < V; v++) {
            if (!visited[v] && adjMatrix[u][v] != NO_EDGE) {
                if (dist[u] != INFINITY && (dist[u] + adjMatrix[u][v] < dist[v])) {
                    dist[v] = dist[u] + adjMatrix[u][v];
                    prev[v] = u;
                    
                    if (minHeap->pos[v] == -1) { 
                         insertHeap(minHeap, dist[v], v);
                    } else {
                         decreaseKey(minHeap, v, dist[v]);
                    }
                    
                    dijkstraProcessTab[step_counter][v].val = dist[v];
                    dijkstraProcessTab[step_counter][v].prev = prev[v];
                }
            }
        }
    }

    if (dist[endNode] == INFINITY) {
        printf("\nKhông có đường đi từ điểm %d đến điểm %d\n", startNode, endNode);
    } else {
        printf("\nĐường đi ngắn nhất từ điểm %d đến điểm %d:\n", startNode, endNode);
        int path[MAX_VERTICES];
        int count = 0;
        int currentNode = endNode;

        while (currentNode != -1) {
            path[count++] = currentNode;
            currentNode = prev[currentNode];
        }
        
        for (int j = count - 1; j >= 0; j--) {
            printf("%d", path[j]);
            if (j > 0) {
                printf(" -> ");
            }
        }
        printf("\n");
        printf("Độ dài đường đi ngắn nhất từ điểm %d đến điểm %d là: %.0lf\n", startNode, endNode, dist[endNode]);
    }

    printf("\nBạn có muốn in quá trình thực hiện không? (y/n): ");
    char print_choice;
    scanf(" %c", &print_choice);
    while ((getchar()) != '\n');
    if (print_choice == 'y') {
        printDijkstraProcess(V, startNode, step_counter);
    }

    freeMinHeap(minHeap);
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

bool bellmanFord(Graph* graph, int src) {
    int V = graph->V;
    int E = graph->E;
    double dist[MAX_VERTICES];
    int parent[MAX_VERTICES];

    src_bellman_global = src;
    Vert_bellman_global = V;
    Ed_bellman_global = E;

    for (int i = 0; i < V; i++) {
        dist[i] = INFINITY;
        parent[i] = -1;
    }
    dist[src] = 0;

    for (int i = 0; i < V; i++) {
        Btab[0][i].dist = (i == src) ? 0.0 : INFINITY;
        Btab[0][i].parent = -1;
        Btab[0][i].changed = false;
    }

    for (int i = 1; i <= V - 1; i++) {
        for (int j = 0; j < V; j++) {
            Btab[i][j].dist = Btab[i-1][j].dist;
            Btab[i][j].parent = Btab[i-1][j].parent;
            Btab[i][j].changed = false;
        }

        bool any_change_in_this_iteration = false;
        for (int j = 0; j < E; j++) {
            int u = graph->edges[j].src;
            int v = graph->edges[j].dest;
            double weight = graph->edges[j].weight;

            if (dist[u] != INFINITY && dist[u] + weight < dist[v]) {
                dist[v] = dist[u] + weight;
                parent[v] = u;
                Btab[i][v].dist = dist[v];
                Btab[i][v].parent = parent[v];
                Btab[i][v].changed = true;
                any_change_in_this_iteration = true;
            }
        }
    }

    for (int j = 0; j < E; j++) {
        int u = graph->edges[j].src;
        int v = graph->edges[j].dest;
        double weight = graph->edges[j].weight;

        if (dist[u] != INFINITY && dist[u] + weight < dist[v]) {
            printf("\nĐồ thị chứa chu trình âm!\n");
            return false;
        }
    }

    printBellmanResult(dist, parent, V, src);
    printf("\nBạn có muốn in quá trình thực hiện không? (y/n): ");
    char print_choice;
    scanf(" %c", &print_choice);
    while ((getchar()) != '\n');
    if (print_choice == 'y') {
        printBellmanProcess(V, src);
    }

    return true;
}

void printBellmanProcess(int V, int startNode) {
    printf("\nBảng quá trình tìm đường đi (Với k là số lần lặp):\n");
    printf("\nk        ");
    for (int i = 0; i < V; i++){
        printf("%14d", i);
    }
    printf("\n_________");
    for (int i = 0; i < V; i++){
        printf("______________");
    }

    printf("\n0        ");
    for (int i = 0; i < V; i++){
        if (Btab[0][i].dist == INFINITY){
            printf("   (INF, -)   ");
        }
        else
            printf("  (%1.0lf, -)   ", Btab[0][i].dist);
    }
    printf("\n");

    for (int k = 1; k < V; k++){
        printf("%-8d ", k);
        for (int j = 0; j < V; j++){
            if (Btab[k][j].dist == INFINITY){
                printf(" (INF, -)   "); 
            } else {
                if (Btab[k][j].dist < 10 && Btab[k][j].dist >= 0) {
                    printf("(%.1lf, %2d)   ", Btab[k][j].dist, Btab[k][j].parent);
                } else if (Btab[k][j].dist < 100 && Btab[k][j].dist >= 0) {
                    printf("(%.1lf, %2d)   ", Btab[k][j].dist, Btab[k][j].parent);
                } else if (Btab[k][j].dist >= 100 || Btab[k][j].dist < 0) {
                    printf("(%.1lf, %2d)  ", Btab[k][j].dist, Btab[k][j].parent); 
                }
                else {
                    printf("(%.1lf, %2d)  ", Btab[k][j].dist, Btab[k][j].parent); 
                }
            }
        }
        printf("\n");
    }
}


void inputGraphFromKeyboard(Graph* graph) {
    int graphType;
    system("cls");
    gotoxy(65,0);
    printf("%sNhập kiểu đồ thị: %sNếu vô hướng nhập 0, có hướng nhập 1 (0/1)? ",YELLOW, WHITE);
    scanf("%d",&graphType);
    while ((getchar()) != '\n');
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
    while ((getchar()) != '\n');
    while (V > MAX_VERTICES){
        gotoxy(65+14,1);
        printf("Lỗi: Số đỉnh vượt quá giới hạn cho phép (%d đỉnh).\n", MAX_VERTICES);
        Sleep(2000);
        gotoxy(65+14,1);
        printf("                                                                  ");
        gotoxy(65+14,1);
        scanf("%d", &V);
        while ((getchar()) != '\n');
    }
    gotoxy(65+14,2);
    scanf("%d", &E);
    while ((getchar()) != '\n');

    while (E > MAX_EDGES) {
        gotoxy(65+14,2);
        printf("Lỗi: Số cạnh vượt quá giới hạn cho phép (%d cạnh).\n", MAX_EDGES);
        Sleep(2000);
        gotoxy(65+14,2);
        printf("                                                                  ");
        gotoxy(65+14,2);
        scanf("%d", &E);
        while ((getchar()) != '\n');
    }

    initGraph(graph, V);
    system("cls");
    gotoxy(65,1);
    printf("Nhập danh sách cạnh (đỉnh đầu, đỉnh cuối, khoảng cách):\n");
    for (int i = 0; i < E; i++) {
        gotoxy(65,i+2);
        int src_edge, dest_edge;
        double weight;
        scanf("%d %d %lf", &src_edge, &dest_edge, &weight);
        while ((getchar()) != '\n');
        if (src_edge >= 0 && src_edge < V && dest_edge < V) {
            addEdge(graph, src_edge, dest_edge, weight);
            if (graphType==0){
                addEdge(graph, dest_edge, src_edge, weight);
            }
        } else {
            gotoxy(65,i+2);
            printf("Cảnh báo: Chỉ số không hợp lệ (%d, %d) cho cạnh %d. Bỏ qua...\n", src_edge, dest_edge, i);
            Sleep(1000);
            gotoxy(65,i+2);
            printf("                                                                    ");
            i--;
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
    printf("Số cạnh: %d\n", graph->E / (graphType == 0 ? 2 : 1));
    gotoxy(65,4);
    printf("Danh sách cạnh:\n");
    if (graphType == 0){
        for (int i = 0; i < graph->E; i+=2) {
            gotoxy(65,i/2+5);
            printf("%d %d %.2lf\n", graph->edges[i].src, graph->edges[i].dest, graph->edges[i].weight);
        }
        gotoxy(65,graph->E/2+5);
    }
    else if (graphType == 1){
        for (int i = 0; i< graph->E; i++){
            gotoxy(65,i+5);
            printf("%d %d %.2lf\n", graph->edges[i].src, graph->edges[i].dest, graph->edges[i].weight);
        }
        gotoxy(65,graph->E+5);
    }

    printf("(y/n)? ");
    char confirm;
    scanf(" %c", &confirm);
    while ((getchar()) != '\n');
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

    int V_file, E_file;
    if (fscanf(file, "%d %d", &V_file, &E_file) != 2) {
        gotoxy(65,0);
        printf("Lỗi đọc số đỉnh và số cạnh từ file.\n");
        fclose(file);
        return 0;
    }

    if (V_file > MAX_VERTICES || E_file > MAX_EDGES) {
        printf("Lỗi: Đồ thị trong file vượt mức cho phép (%d đỉnh, %d cạnh).\n", MAX_VERTICES, MAX_EDGES);
        fclose(file);
        return 0;
    }

    initGraph(graph, V_file);

    for (int i = 0; i < E_file; i++) {
        int src_edge, dest_edge;
        double weight;
        if (fscanf(file, "%d %d %lf", &src_edge, &dest_edge, &weight) != 3) {
             printf("Lỗi không thể đọc cạnh %d từ file.\n", i);
             continue;
        }
        if (src_edge >= 0 && src_edge < V_file && dest_edge >= 0 && dest_edge < V_file) {
             addEdge(graph, src_edge, dest_edge, weight);
        } else {
             printf("Cảnh báo: Chỉ số không hợp lệ (%d, %d) cho cạnh %d trong file. Bỏ qua...\n", src_edge, dest_edge, i);
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
        fprintf(file, "%d %d %.2lf\n",
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
    int c;
    while ((c = getchar()) != '\n' && c != EOF);
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

void urchoice(Graph *graph) {
    int algo_choice;
    clock_t start, end;
    double time_taken_Dijkstra, time_taken_BF;
    system("chcp 65001 > nul");
    SetConsoleOutputCP(65001);
    if (graph->V <= 0) {
        printf("Không có đồ thị để thực hiện thuật toán.\n");
        pressAnyKeyToContinue();
        return;
    }

    do {
        chooseAlgorithm();
        scanf("%d", &algo_choice);
        while ((getchar()) != '\n');
        int temp_firstNode, temp_lastNode, temp_src;
        switch(algo_choice) {
            case 1:
                system("cls");
                printf("Nhập điểm bắt đầu và điểm kết thúc (0-%d): ", graph->V - 1);
                scanf("%d %d", &temp_firstNode, &temp_lastNode);
                while ((getchar()) != '\n');

                if (temp_firstNode < 0 || temp_firstNode >= graph->V || temp_lastNode < 0 || temp_lastNode >= graph->V) {
                    printf("Lỗi: Đỉnh bắt đầu hoặc kết thúc không hợp lệ.\n");
                    break;
                }
                start = clock();
                dijkstra(adjMatrix_dijkstra_global, graph->V, temp_firstNode, temp_lastNode);
                end = clock();
                printf("\nThời gian thực hiện thuật toán: %.10f giây\n", (double)(end - start) / CLOCKS_PER_SEC);
                break;
            case 2:
                system("cls");
                printf("\nNhập điểm bắt đầu (0-%d): ", graph->V-1);
                scanf("%d", &temp_src);
                while ((getchar()) != '\n');

                if (temp_src < 0 || temp_src >= graph->V) {
                     printf("Lỗi: Điểm bắt đầu không hợp lệ.\n");
                     break;
                }
                start = clock();
                bellmanFord(graph, temp_src);
                end = clock();
                printf("\nThời gian thực hiện thuật toán: %.20f giây\n", (double)(end - start) / CLOCKS_PER_SEC);
                break;
            case 3:
                system("cls");
                printf("So sánh thời gian thực hiện giữa Dijkstra và Bellman-Ford:\n");
                printf("Thuật toán Dijkstra sẽ chạy từ đỉnh 0 đến đỉnh %d\n", graph->V-1);
                printf("Thuật toán Bellman-Ford sẽ chạy từ đỉnh 0 đến tất cả các đỉnh còn lại.\n");
                
                printf("\n--- Chạy Dijkstra ---\n");
                start = clock();
                dijkstra(adjMatrix_dijkstra_global, graph->V, 0, graph->V-1);
                end = clock();
                time_taken_Dijkstra = (double)(end - start) / CLOCKS_PER_SEC;

                printf("\n--- Chạy Bellman-Ford ---\n");
                start = clock();
                bellmanFord(graph, 0);
                end = clock();
                time_taken_BF = (double)(end - start) / CLOCKS_PER_SEC;

                printf("\nThời gian thực hiện thuật toán Dijkstra: %.10f giây\n", time_taken_Dijkstra);
                printf("Thời gian thực hiện thuật toán Bellman-Ford: %.10f giây\n", time_taken_BF);

                if (time_taken_BF > time_taken_Dijkstra) {
                    printf("Thời gian thực hiện thuật toán Dijkstra nhanh hơn Bellman-Ford %.10lf giây.\n", time_taken_BF - time_taken_Dijkstra);
                } else if (time_taken_Dijkstra > time_taken_BF) {
                    printf("Thời gian thực hiện thuật toán Bellman-Ford nhanh hơn Dijkstra %.10lf giây.\n", time_taken_Dijkstra - time_taken_BF);
                } else {
                    printf("Thời gian thực hiện của cả hai thuật toán là như nhau.\n");
                }
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
    printCentered(67, 1, 58, 2, "Dijkstra & Bellman-Ford", MINT_GREEN);
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

void freeMatrix(double **a, int rows){
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
    for (int i = 0; i < capacity; i++) {
        minHeap->pos[i] = -1;
    }
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
        HeapNode emptyNode = {-1, INFINITY};
        return emptyNode;
    }

    HeapNode root = minHeap->nodes[0];
    HeapNode lastNode = minHeap->nodes[minHeap->size - 1];
    minHeap->nodes[0] = lastNode;

    minHeap->pos[root.vertex] = -1;
    minHeap->pos[lastNode.vertex] = 0;

    minHeap->size--;
    minHeapify(minHeap, 0);

    return root;
}

void decreaseKey(MinHeap* minHeap, int vertex, double distance) {
    if (minHeap->pos[vertex] == -1) {
        return;
    }

    int i = minHeap->pos[vertex];
    minHeap->nodes[i].distance = distance;

    while (i > 0 && minHeap->nodes[i].distance < minHeap->nodes[(i - 1) / 2].distance) {
        minHeap->pos[minHeap->nodes[i].vertex] = (i - 1) / 2;
        minHeap->pos[minHeap->nodes[(i - 1) / 2].vertex] = i;
        
        swapHeapNodes(&minHeap->nodes[i], &minHeap->nodes[(i - 1) / 2]);
        i = (i - 1) / 2;
    }
}

void insertHeap(MinHeap* minHeap, double distance, int vertex) {
    if (minHeap->size == minHeap->capacity) {
        printf("Heap is full. Cannot insert.\n");
        return;
    }
    minHeap->nodes[minHeap->size].distance = distance;
    minHeap->nodes[minHeap->size].vertex = vertex;
    minHeap->pos[vertex] = minHeap->size;
    minHeap->size++;
    decreaseKey(minHeap, vertex, distance);
}


void freeMinHeap(MinHeap* minHeap) {
    if (minHeap) {
        free(minHeap->pos);
        free(minHeap->nodes);
        free(minHeap);
    }
}

void initQueue(CircularQueue* q) {
    q->front = 0;
    q->rear = -1;
    q->count = 0;
}

bool isQueueEmpty(CircularQueue* q) {
    return q->count == 0;
}

bool isQueueFull(CircularQueue* q) {
    return q->count == QUEUE_SIZE;
}

void enqueue(CircularQueue* q, Edge item) {
    if (isQueueFull(q)) {
        printf("Queue is full. Cannot enqueue.\n");
        return;
    }
    q->rear = (q->rear + 1) % QUEUE_SIZE;
    q->data[q->rear] = item;
    q->count++;
}

Edge dequeue(CircularQueue* q) {
    Edge item = {-1, -1, -1};
    if (isQueueEmpty(q)) {
        printf("Queue is empty. Cannot dequeue.\n");
        return item;
    }
    item = q->data[q->front];
    q->front = (q->front + 1) % QUEUE_SIZE;
    q->count--;
    return item;
}

void printBellmanResult (double dist[MAX_VERTICES], int parent[MAX_VERTICES], int V, int src) {
    printf("\nKhoang cach va duong di tu dinh %d:\n", src);
    for (int i = 0; i < V; i++) {
        if (dist[i] == INFINITY) {
            printf("Dinh %d: Khong the den duoc\n", i);
        } else {
            printf("Dinh %d: Khoang cach = %.1lf, Duong di: ", i, dist[i]);
            int path[MAX_VERTICES];
            int count = 0;
            int current = i;

            while (current != -1) {
                path[count++] = current;
                current = parent[current];
            }
            for (int j = count-1; j >= 0; j--) {
                printf("%d", path[j]);
                if (j > 0) printf(" -> ");
            }
            printf("\n");
        }
    }
}
