* Compiling and Running *
g++ *.cpp -o main -Wall
./main


* Valgrind *
sudo apt install valgrind
valgrind -s --leak-check=full --show-leak-kinds=all --track-origins=yes ./main


INPUTS:
    (- PARÂMETROS DAS METAHEURÍSTICAS)
    
    - LISTA DE ESTAÇÕES CANDIDATAS
        -> CANDIDATA QUER DIZER QUE A ESTAÇÃO PODE SER USADA OU NÃO NUMA SOLUÇÃO
    
    - LISTA DE REQUISIÇÕES
        -> ORIGEM
        -> DESTINO
        -> HORÁRIO DE INÍCIO DA VIAGEM
    

    - GRAFO DIRECIONADO
        -> Existem três tipos de vértices:
            * Estação:
                - Arestas somente para vértices do tipo Origem.
            
            * Origem:
                - Aresta única para seu vértice Destino correspondente.
            
            * Destino:
                - Arestas para quaisquer vértices do tipo Origem e Estação.

        -> Cada aresta tem :
            * Custo de viagem constante (unitário)
            * Tempo de viagem associado dependente da distância

    - Insights:
        -> A matriz de feromônios é referenciada a partir de valores entre arestas (i,j)
        -> Construir uma matriz de distâncias entre os pontos (vértices do grafo) 
        -> A matriz de ferômonios usará as posições (i,j) como referência e fará 
        updates à quantidade de feromônios a cada iteração do ACO.
        -> Cada célula da matriz será uma tupla (distância(i,j), tau(t))
        -> Caminhos (i,j) impossíveis possuirão distância INF e não serão atualizados
        OU uso uma lista de adjacências para reduzir o uso de memória já que existem
        muitos movimentos impossíveis.
        -> Fixar, em cada candidato, as estações candidatas habilitadas
            * Randomizar quantidade e índices.
        -> A adição de um veículo extra não entra como "movimento" já que está ligado
        à viabilidade do modelo e não a um movimento aleatório numa aresta.

Construção da Solução: FORMIGA

Uma formiga seleciona arestas válidas para compor sua solução.

Função probabilística para determinar se uma formiga k, na iteração t, usará ou não determinada aresta.

p(i,j)_k_t = ( [Tau(i,j)_t]^alpha * [Eta(i,j)]^beta ) / ( Sum([Tau(i,j)_t]^alpha * [Eta(i,j]^beta) )

(Se j pertence ao conjunto dos vértices não visitados por k)
(Caso contrário, p(i,j)_k_t = 0)



Matriz de feromônios --> Lista de Adjacência já que existem muitos movimentos proibidos.

Tau(t+1) -> Atualiza a matriz para a iteração t+1 baseado em t e nas informações heurísticas.

Tau(t+1) = (1-ro) * Tau(i,j)_t + ro * delta_Tau(i,j)

delta_Tau(i,j) = 1/f(S)     Se (i,j) pertence à solução S
delta_Tau(i,j) = 0          Caso contrário


Evaporação:
Tau(t)      = (1-ro) * Tau(t)
Tau(t+1)    = 








INPUT

Se tenho estipulado que nenhuma viagem tem duração superior a 45 min,
Então posso determinar a distância máxima entre uma origem e um destino:
    velocidade média = (deslocamento)/(tempo)
    Sendo a velocidade média = 35 km/h:
    35 = (deslocamento máximo) / (45/60);
    deslocamento máximo = 26,25 km.



/*
    Each iteration must randomize a new valid node
    Valid nodes are characterized by:
        1_ valid movement: already processed at the beginning
            * station -> request
            * request -> request
            * request -> station
        2_ unanswered requests
            * is_used == false
        3_ time-feasible
            * is_on_time == true
            * true for all stations
            * calculated for requests:
                . time_of_vehicle: tov
                . time_of_cruise: toc  (different for each type of node)
                . time_of_request_start: tors
                . tov + toc <= tors + eps
        4_ energy-feasible
            * has_energy -> tricky considering recharge
                . current_energy_of_vehicle: ceov
                . energy_to_complete_trip: etct
                . if ceov - etct < minimum_energy
                    check for recharge-feasibility
                    implies in new time-feasibility check

    After selecting only valid nodes, proceed to randomly choose one
    
    This kind of check will guarantee that only stations will remain
    once a car is unable to complete more trips.
    Checking each node should take O(1)
    */


/*
        Three kinds of trips:
        1. Request to Station:
            - A: request destination (current_vertex)
            - B: station

        2. Station to Request:
            - A: station (current_vertex)
            - B: request origin
            - C: request destination

        3. Request X to Request Y:
            I) Without recharging
                - A: request X destination (current_vertex)
                - B: request Y origin
                - C: request Y destination
            II) With recharge
                - A: request X destination (current_vertex)
                - B: station S
                - C: request Y origin
                - D: request Y destination
    */