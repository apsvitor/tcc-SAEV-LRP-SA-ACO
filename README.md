


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