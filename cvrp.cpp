#include<vector>
#include<iostream>
#include<fstream>
#include<cstring>
#include<cmath>
#include<string>
#include<limits>
#include<malloc.h>


class Node{
    public:
        int nodeId;//id korisnika
        int x,y;// koordinate korisnika
        int demand;//potraznja korisnika
        bool isRouted;//da li je korsinik u ruti nekog vozila
        bool isDepot;//da li je ovaj cvor skladiste

        Node(){//default konstruktor
            nodeId=-1;
            demand=0;
        }

        Node(int depotX, int depotY){ //Konstruktor za skladiste, uvek ima id=0 i potraznju=0
            x=depotX;
            y=depotY;
            nodeId=0;
            demand=0;
            isDepot=true;
        }

        Node(int id,int x_,int y_, int demand_){ //Konstruktor za musterije
            nodeId=id;
            x=x_;
            y=y_;
            demand=demand_;
            isRouted=false;
            isDepot=false;
        }
};

class Vehicle{
    public:
        int vehicle_id;//id vozila
        std::vector<Node> Route;//ruta korisnika koje vozilo obilazi
        int capacity;//nosivost vozila
        int load;//trenutno opterecenje vozila
        int curLoc;//trenutna lokacija vozila

        Vehicle(){//default konstruktor
            vehicle_id=-1;
        }

        Vehicle(int id, int cap){//konstrukor za vozila,na pocetku load =0 i trenutna lokacija je 0 (skladiste)
            vehicle_id=id;
            capacity=cap;
            load=0;
            curLoc=0;
            Route.clear();
        }

        void addNode(Node customer){//Dodaje musteriju na kraj rute vozila
            Route.push_back(customer);
            load += customer.demand;
            curLoc=customer.nodeId;
        }

        bool checkIfFits(int dem){ //Proverava da li ima dovoljno mesta u vozilu za dem
            return ((load+dem) <= capacity);
        }
};

class Solution{
    public:
        int noOfVehicles;
        int noOfCustomers;
        std::vector<Vehicle> vehicles;
        double cost;

        std::vector<Vehicle> vehiclesForBestSolution;
        double bestSolutionCost;

        Solution(int custNum,int vehNum, int vehCap){
            noOfCustomers=custNum;
            noOfVehicles=vehNum;
            cost=0;
            
            
            vehicles.resize(vehNum);
            vehiclesForBestSolution.resize(vehNum);
            
            for(int i=0;i< vehNum;i++){
                
                Vehicle tmp1(i+1,vehCap);
                vehicles[i]=tmp1;
                
                Vehicle tmp2(i+1,vehCap);
                vehiclesForBestSolution[i]=tmp2;
            }
        }

        bool UnassignedCustomerExists(std::vector<Node> nodes){//Proverava da li postoje musterije koje nisu posecene
            for(size_t i=1;i<nodes.size();i++){
                if(!nodes[i].isRouted){
                    return true;
                }
            }
            return false;
        }

        void GreedySolution(std::vector<Node> nodes, std::vector<std::vector<double>> costMatrix){
            double candCost,endCost;
            int vehIndex=0;
            

            while(UnassignedCustomerExists(nodes)){
                
                int custIndex =-1;
                double minCost = std::numeric_limits<double>::max();

                if(vehicles[vehIndex].Route.empty()){
                    vehicles[vehIndex].addNode(nodes[0]);
                }
                
                for(int i=1;i<=noOfCustomers;i++){
                    if(!nodes[i].isRouted){
                        
                        if(vehicles[vehIndex].checkIfFits(nodes[i].demand)){
                            
                            candCost = costMatrix[vehicles[vehIndex].curLoc][i];
                            
                            if(minCost > candCost){
                                minCost=candCost;
                                custIndex=i;
                            }
                        }
                    }
                }
                
                if(custIndex==-1){//nema vise mesta ni za jednu musteriju za trenutno vozilo
                    if(vehIndex+1<vehicles.size()){//imamo jos praznih vozila
                        if(vehicles[vehIndex].curLoc!=0){//zavrsi rutu tako sto ce se vozilo vratiti u skladiste
                            endCost=costMatrix[vehicles[vehIndex].curLoc][0];
                            vehicles[vehIndex].addNode(nodes[0]);
                            cost += endCost;
                        }
                        vehIndex +=1; // predji na sledece vozilo
                    }
                    else{//nemamo vise praznih vozila, nismo nasli resenje problema
                        std::cerr<< "Nismo nasli resenje problema. GreedySolution\n";
                        std::exit(0);
                    }
                }else{//dodajemo najblizeg kandidata u rutu trenutnog vozila
                    vehicles[vehIndex].addNode(nodes[custIndex]);
                    nodes[custIndex].isRouted=true;
                    cost+=minCost;
                }


            }

            endCost = costMatrix[vehicles[vehIndex].curLoc][0];
            vehicles[vehIndex].addNode(nodes[0]);
            cost+=endCost;
        }

        void TabuSearch(int tabu_horizon,std::vector<std::vector<double>> costMatrix){
            
            std::vector<Node> routeFrom;
            std::vector<Node> routeTo;
            
            int movingNodeDemand=0;
            size_t vehIndeksFrom, vehIndexTo;
            double bestNcost,neighbourCost;
            int swapIndexA=-1,swapIndexB=-1,swapRouteFrom=-1,swapRouteTo=-1;

            int MAX_ITERATIONS=100000;
            int it_num=0;

            int dimensionCustomer = costMatrix[1].size();
            std::vector<std::vector<int>> tabuMatirx(dimensionCustomer+1);

            for(int i=0;i<dimensionCustomer+1;i++){
                tabuMatirx[i].resize(dimensionCustomer+1);
            }

            bestSolutionCost=cost;

            bool flag = false;

            bool termination=false;
            while(!termination){
                it_num++;
                bestNcost=std::numeric_limits<double>::max();
                flag = false;
                swapIndexA=-1,swapIndexB=-1,swapRouteFrom=-1,swapRouteTo=-1;
                for(vehIndeksFrom=0;vehIndeksFrom<vehicles.size();vehIndeksFrom++){
                    
                    routeFrom=vehicles[vehIndeksFrom].Route;
                    size_t routeFromLength= routeFrom.size();
                    
                    for(size_t i=1;i<routeFromLength-1;i++){//pocinjemo od 1 jer nije moguce pomeriti skladiste
                       
                        for(vehIndexTo=0;vehIndexTo<vehicles.size();vehIndexTo++){
                            routeTo = vehicles[vehIndexTo].Route;
                            size_t routeToLength = routeTo.size();
                            for(size_t j=0;j<routeToLength-1;j++){//moramo da se zavrsva sa skladistem
                                
                                movingNodeDemand=routeFrom[i].demand;

                                if((vehIndeksFrom==vehIndexTo) || vehicles[vehIndexTo].checkIfFits(movingNodeDemand)){
                                    //ako premestamo unutar iste rute ne moramo da proveravamo da li ima mesta

                                    if(((vehIndeksFrom==vehIndexTo)&&((j==i)||(j==i-1)))==false){//samo potezi koji menjaju cenu
                                        double minusCost1=costMatrix[routeFrom[i-1].nodeId][routeFrom[i].nodeId];
                                        double minusCost2=costMatrix[routeFrom[i].nodeId][routeFrom[i+1].nodeId];
                                        double minusCost3=costMatrix[routeTo[j].nodeId][routeTo[j+1].nodeId];

                                        double addedCost1 = costMatrix[routeFrom[i-1].nodeId][routeFrom[i+1].nodeId];
                                        double addedCost2 = costMatrix[routeTo[j].nodeId][routeFrom[i].nodeId];
                                        double addedCost3 = costMatrix[routeFrom[i].nodeId][routeTo[j+1].nodeId];

                                        //Provera da li je potez tabu
                                        bool is_tabu = ((tabuMatirx[routeFrom[i-1].nodeId][routeFrom[i+1].nodeId] != 0) ||
                                                       (tabuMatirx[routeTo[j].nodeId][routeFrom[i].nodeId] != 0) ||
                                                       (tabuMatirx[routeFrom[i].nodeId][routeTo[j+1].nodeId] !=0));                                    

                                        neighbourCost = addedCost1+addedCost2+addedCost3 - minusCost1 -minusCost2 - minusCost3;

                                        if((neighbourCost < bestNcost) && (vehicles[vehIndeksFrom].load-movingNodeDemand>=0) && (vehicles[vehIndexTo].load+movingNodeDemand<=vehicles[vehIndexTo].capacity) && !is_tabu){
                                            flag=true;
                                            bestNcost=neighbourCost;
                                            swapIndexA=i;
                                            swapIndexB=j;
                                            swapRouteFrom=vehIndeksFrom;
                                            swapRouteTo=vehIndexTo;
                                        }
                                    }
                                }
                            }
                        }
                    }
                }

                for(int i=0;i<tabuMatirx[0].size();i++){
                    for(int j=0;j<tabuMatirx[0].size();j++){
                        if(tabuMatirx[i][j]>0){
                            tabuMatirx[i][j]--;
                        }
                    }
                }
                
                routeFrom = vehicles[swapRouteFrom].Route;
                routeTo = vehicles[swapRouteTo].Route;
                vehicles[swapRouteFrom].Route.clear();
                vehicles[swapRouteTo].Route.clear();

                Node swapNode = routeFrom[swapIndexA];

                int nodeIdBefore = routeFrom[swapIndexA-1].nodeId;
                int nodeIdAfter = routeFrom[swapIndexA+1].nodeId;
                int nodeID_f = routeTo[swapIndexB].nodeId;
                int nodeID_g = routeTo[swapIndexB+1].nodeId;

                tabuMatirx[nodeIdBefore][swapNode.nodeId] = tabu_horizon;
                tabuMatirx[swapNode.nodeId][nodeIdAfter] = tabu_horizon;
                tabuMatirx[nodeID_f][nodeID_g] = tabu_horizon;
                
                routeFrom.erase(routeFrom.begin()+swapIndexA);
                

                if(swapRouteFrom==swapRouteTo){
                    routeTo.erase(routeTo.begin()+swapIndexA);
                    if(swapIndexA<swapIndexB){
                        auto itPos = routeTo.begin()+swapIndexB;
                        routeTo.insert(itPos,swapNode);
                    }else{
                        auto itPos = routeTo.begin()+swapIndexB+1;
                        routeTo.insert(itPos,swapNode);
                    }
                }else{
                    auto itPos = routeTo.begin()+swapIndexB+1;
                    routeTo.insert(itPos,swapNode);
                }
                
                vehicles[swapRouteFrom].Route = routeFrom;
                vehicles[swapRouteTo].Route = routeTo;
                if(swapRouteFrom!=swapRouteTo){
                    vehicles[swapRouteFrom].load -= swapNode.demand;
                    vehicles[swapRouteTo].load += swapNode.demand;
                }
                

                cost += bestNcost;

                if(cost < bestSolutionCost){
                    saveBestSolution();
                }
                if(it_num == MAX_ITERATIONS){
                    termination=true;
                }

            }

            vehicles = vehiclesForBestSolution;
            cost = bestSolutionCost;

            
        }

        void saveBestSolution(){
            bestSolutionCost = cost;
            for(int i=0;i<noOfVehicles;i++){
                vehiclesForBestSolution[i].Route.clear();
                if(!vehicles[i].Route.empty()){
                    for(size_t j=0;j<vehicles[i].Route.size();j++){
                        Node n = vehicles[i].Route[j];
                        vehiclesForBestSolution[i].Route.push_back(n);
                    }
                }
            }
        }

        void solutionPrint(std::string solutionLabel){
            std::cout << "======================================================\n" << solutionLabel << std::endl;

            for(int i=0;i<noOfVehicles;i++){
                if(!vehicles[i].Route.empty()){
                    std::cout << "Vehicle " << i << ":\n";
                    int routSize = vehicles[i].Route.size();
                    for(int j=0;j<routSize;j++){
                        if(j==routSize-1){
                            std::cout<< vehicles[i].Route[j].nodeId;
                        }else{
                            std::cout << vehicles[i].Route[j].nodeId << "->";
                        }
                    }
                    std::cout<<std::endl;
                }
            }
            std::cout<<"\nSolution cost " << cost << std::endl; 
        }

};

class Cvrp {
public:
    
    
    int noOfCustomers;
    
    int noOfVehicles;

    int vehicleCapcity;

    std::vector<Node> customers;
    
    std::vector<int> demands;

    std::vector<std::vector<double>> distances;

    void solve(){
        
        int tabu_horizon=10;
        
        Solution s = Solution(noOfCustomers,noOfVehicles,vehicleCapcity);
        
        s.GreedySolution(customers,distances);
    
        s.solutionPrint("Greedy Solution");
    
        s.TabuSearch(10,distances);
        s.solutionPrint("Tabu Search");
    
    }

    Cvrp(const char* strNoOfVehicles) {
        noOfVehicles = atoi(strNoOfVehicles);
    }

    void loadExample(const std::string& fileName) {
        loadData(fileName);
        if (noOfVehicles == 0) {
            noOfVehicles = getVehNumber(fileName);
        }
    }

    char* trim(char* token){
        char* t;
        for(t = token + strlen(token); --t >= token; )//izbrisi beline sa kraja
            if (*t == ' ' || *t=='\t' || *t=='\r' || *t=='\a')
              *t = '\0';
            else
                break;
            //izbrisi beline sa pocetka
        for(t = token; t < token + 100; ++t)
            if(*t != ' ')
                break;
        
        return t;
    }

    void loadData(const std::string& fileName) {
        
        std::ifstream infile(fileName);
        if (!infile.is_open()) {
            throw std::runtime_error("Greska pri otvaranju fajla");
        }

        std::string str;
        char* token;
        char* line;
        int noOfNodes;
        
        while (true) {
            
            std::getline(infile, str);
            
            line = strdup(str.c_str());
            
            token = trim(strtok(line, " :"));
            
            
            if(strcmp(token,"DIMENSION")==0){       
                token=trim(strtok(NULL," :"));
                noOfNodes = atoi(token);
                noOfCustomers = noOfNodes - 1;
            }
            else if (strcmp(token, "CAPACITY") == 0) {
                token=trim(strtok(NULL," :"));
                vehicleCapcity = atoi(token);
            }
            else if (strcmp(token, "EDGE_WEIGHT_TYPE") == 0) {
                token=trim(strtok(NULL," :"));
                
                if (strcmp(token, "EUC_2D") != 0) {
                    throw std::runtime_error("Podrzan je samo EUC_2D EDGE_WEIGHT_TYPE");
                }
                
            }
            else if (strcmp(token, "NODE_COORD_SECTION") == 0) {
                break;
            }
        }
        
        std::vector<int> nodesX (noOfCustomers);
        std::vector<int> nodesY (noOfCustomers);
        int depotX = 0, depotY = 0;
        for (int i = 1; i <= noOfNodes; i++) {
            int id = 0;
            infile >> id;
            
            if (id != i) {
                throw std::runtime_error("Neocekivani indeks1");
            }
            if (i == 1) {
                infile >> depotX;
                infile >> depotY;
            }
            else {
                infile >> nodesX[i-1];
                infile >> nodesY[i-1];
            }
        }


        std::getline(infile, str);
        std::getline(infile, str);
        line = strdup(str.c_str());
        token = strtok(line, " :");
        
        if (strcmp(token, "DEMAND_SECTION") != 0) {
            throw std::runtime_error("Ocekivan token DEMAND_SECTION");
        }

        demands.resize(noOfCustomers+2);
        for (int i = 1; i <= noOfNodes; i++) {
            int id = 0;
            infile >> id;
            if (id != i) {
                throw std::runtime_error("Neocekivani indeks");
            }
            int demand;
            infile >> demand;
            if (i == 1) {
                if (demand != 0) {
                    throw std::runtime_error("Potraznja za skladiste mora biti 0");
                }
            }
            else {
                demands[i-1] = demand;
            }
        }
        
        std::getline(infile, str);
        std::getline(infile, str);
        line = strdup(str.c_str());
        token = strtok(line, " :");
        if (strcmp(token, "DEPOT_SECTION") != 0) {
            throw std::runtime_error("Ocekivan token DEPOT_SECTION");
        }

        int depotId = 0;
        infile >> depotId;
        if (depotId != 1) {
            throw std::runtime_error("Id skladista treba da bude 1");
        }

        int endOfDepotSection = 0;
        infile >> endOfDepotSection;
        if (endOfDepotSection != -1) {
            throw std::runtime_error("Ocekivano je samo jedno skladiste");
        }
        
        customers.resize(noOfCustomers+1);
        for(int i=0;i<=noOfCustomers;i++){
            
            if(i==0){
                Node tmp(depotX,depotY);
                customers[i]=tmp;
            }else{
                Node tmp(i,nodesX[i],nodesY[i],demands[i]);
                customers[i]=tmp;
            }
        }
        
        calculateDistanceMatrix();
        
        infile.close();
    }

    void calculateDistanceMatrix() {
        
        distances.resize(noOfCustomers+1);
        for (int i = 0; i <= noOfCustomers; i++) {
            distances[i].resize(noOfCustomers+1);
        }
        
        for (int i = 0; i <= noOfCustomers; i++) {
            
            for (int j = 0; j <= noOfCustomers; j++) {
                
                auto dist = calculateDistance(customers[i].x, customers[i].y, customers[j].x, customers[j].y);
                
                distances[i][j] = dist;
                distances[j][i] = dist;
            }
        }

        

    }

    double calculateDistance(int xi, int yi, int xj, int yj) {
        return round(sqrt(pow((double)xi - xj, 2) + pow((double)yi - yj, 2)));
    }

    int getVehNumber(const std::string& fileName) {
        size_t position = fileName.find("-k");
        if (position != std::string::npos) {
            std::string vehNumString = fileName.substr(position + 2,1);
            return atoi(vehNumString.c_str());
        }
        throw std::runtime_error("Greska: nije pronadjen broj vozila u imenu fajla, prosledite ga kao argument komandne linije");
        return -1;
    }
};

int main(int argc, char** argv) {
    if (argc < 2) {
        std::cerr << "cvrp ulazniFajl [brVozila]" << std::endl;
        return 1;
    }
    const char* inputFile = argv[1];
    const char* vehNumber = argc > 2 ? argv[2] : "0";

    try
    {
        
        Cvrp model = Cvrp(vehNumber);
        
        model.loadExample(inputFile);
        model.solve();
        
    }
    catch(const std::exception& e)
    {
        std::cerr << e.what() << std::endl;
    }
    


    return 1;
}


