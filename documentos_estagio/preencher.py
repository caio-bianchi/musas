import sys, calendar, datetime
sys.path.insert(0,'/private/tmp/estagio-python')
from docx import Document
from docx.shared import Cm, Pt
from docx.enum.text import WD_ALIGN_PARAGRAPH
from docx.oxml import OxmlElement
from docx.oxml.ns import qn
from pathlib import Path
OUT=Path(__file__).parent
NAME='Caio Bianchi Nocrato Gomes'
LAB='Laboratório de Comando e Controle e Defesa Cibernética (LAB-C2DC)'
COURSE='Engenharia de Computação'
SUP='Lourenço Alves Pereira Júnior'
ITA='Juliana de Melo Bezerra'
PERIOD='29/06/2026 a 08/10/2026'
SCHEDULE={}
week=datetime.date(2026,6,29)
for w in range(15):
 for offset in ([0,2,3] if w==14 else [0,2,4]):
  day=week+datetime.timedelta(weeks=w,days=offset)
  if day==datetime.date(2026,9,7):day=datetime.date(2026,9,8)
  SCHEDULE[day]=5
assert len(SCHEDULE)==45 and sum(SCHEDULE.values())==225

def doc(size=11):
 d=Document(); s=d.sections[0]; s.page_width=Cm(21);s.page_height=Cm(29.7)
 s.top_margin=Cm(1.8);s.bottom_margin=Cm(1.7);s.left_margin=Cm(2.2);s.right_margin=Cm(2)
 n=d.styles['Normal'];n.font.name='Arial';n.font.size=Pt(size);n.paragraph_format.space_after=Pt(6)
 for k in ['Heading 1','Heading 2','Heading 3']:
  d.styles[k].font.name='Arial';d.styles[k].font.color.rgb=None
 f=s.footer.paragraphs[0];f.alignment=WD_ALIGN_PARAGRAPH.CENTER
 r=f.add_run('ITA • Estágio Curricular Obrigatório | ');r.font.size=Pt(8)
 fld=OxmlElement('w:fldSimple');fld.set(qn('w:instr'),'PAGE');f._p.append(fld)
 return d

def p(d,t='',bold=False):
 x=d.add_paragraph(); x.add_run(t).bold=bold;return x

def title(d,t):
 p(d,'INSTITUTO TECNOLÓGICO DE AERONÁUTICA',True).alignment=WD_ALIGN_PARAGRAPH.CENTER
 p(d,t,True).alignment=WD_ALIGN_PARAGRAPH.CENTER

def info(d):
 for t in [f'Estagiário: {NAME}',f'Curso: {COURSE} | Turma: 2026',f'Empresa/Instituição: {LAB}',f'Supervisor na instituição: {SUP}',f'Supervisora no ITA: {ITA}',f'Período: {PERIOD}']:p(d,t)

def signatures(d):
 t=d.add_table(rows=2,cols=2)
 vals=[NAME+'\nAluno',SUP+'\nOrientador/Supervisor da instituição',ITA+'\nOrientadora/Supervisora do ITA','Coordenador do Curso de Engenharia de Computação/ITA']
 for c,v in zip([c for row in t.rows for c in row.cells],vals):
  c.text='\n____________________________\n'+v
  for a in c.paragraphs:
   a.alignment=WD_ALIGN_PARAGRAPH.CENTER
   for r in a.runs:r.font.size=Pt(9)

# Avaliação: estrutura e critérios do formulário fornecido.
a=doc(9);title(a,'AVALIAÇÃO DE ESTÁGIO');info(a)
p(a,'Resumo das atividades desenvolvidas:',True)
p(a,'Desenvolvimento de uma plataforma modular de simulação UTM em Python, com modelos de UAV, gestão de planos de voo, espaço aéreo tridimensional, obstáculos e geofences. Integração de rastreamento, previsão de trajetórias, detecção e resolução estratégica e tática de conflitos. Modelagem de vento, ruído de sensores e atrasos; elaboração de cenários de teste, persistência em SQLite, métricas e visualizações para análise dos experimentos.')
criteria=[('Qualidade do trabalho','Qualidade dos trabalhos executados, considerando as condições oferecidas e tarefas em nível universitário de Engenharia.'),('Interesse e Aprendizado Prático','Interesse, dedicação, capacidade e esforço de aprendizagem demonstrados.'),('Planejamento e Organização','Planejamento das atividades, rendimento e atendimento de prazos.'),('Engenhosidade/Criatividade','Capacidade de sugerir, projetar ou executar modificações e inovações que beneficiem a instituição.'),('Iniciativa e Independência','Busca de novas soluções sem prévia orientação.'),('Conhecimentos técnicos','Adequação dos conhecimentos à execução das tarefas.'),('Cooperação e trabalho em equipe','Disponibilidade para contribuir e integração à equipe.'),('Responsabilidade','Cumprimento das atividades dispensando coordenação constante.'),('Sociabilidade','Capacidade de relacionamento interpessoal.'),('Liderança','Capacidade de direção e coordenação.'),('Pontualidade','Atendimento dos horários a que está sujeito.')]
t=a.add_table(rows=1,cols=6);t.style='Table Grid'
for c,v in zip(t.rows[0].cells,['ITEM AVALIADO','FRACO','REGULAR','BOM','ÓTIMO','NÃO OBSERVADO']):c.text=v
for k,v in criteria:
 row=t.add_row();row.cells[0].text=k+': '+v
for row in t.rows:
 for i,c in enumerate(row.cells):
  c.width=Cm(8.5 if i==0 else 1.5)
  for x in c.paragraphs:
   x.paragraph_format.space_after=Pt(3)
   for r in x.runs:r.font.size=Pt(8)
p(a,'Observações do supervisor: ________________________________________________________')
p(a,'Critérios de avaliação reservados ao supervisor. Data da avaliação: ____/____/________.')
signatures(a)
a.save(OUT/'Ficha_de_Avaliacao_Caio_Bianchi.docx')

# Relatório: mantém as seções do modelo e desenvolve o conteúdo com evidências do projeto.
d=doc();title(d,'RELATÓRIO FINAL DE ESTÁGIO CURRICULAR OBRIGATÓRIO')
p(d,'CURSO DE ENGENHARIA DE COMPUTAÇÃO',True).alignment=WD_ALIGN_PARAGRAPH.CENTER
for _ in range(4):p(d)
p(d,'Plataforma modular de simulação de serviços UTM e gerenciamento de conflitos',True).alignment=WD_ALIGN_PARAGRAPH.CENTER
p(d,NAME).alignment=WD_ALIGN_PARAGRAPH.CENTER
p(d,LAB).alignment=WD_ALIGN_PARAGRAPH.CENTER
p(d,'Período: '+PERIOD).alignment=WD_ALIGN_PARAGRAPH.CENTER
p(d,'São José dos Campos — outubro de 2026').alignment=WD_ALIGN_PARAGRAPH.CENTER
d.add_page_break();d.add_heading('FOLHA DE APROVAÇÃO',0)
p(d,'Relatório Final de Estágio Curricular aceito em ____/____/________ pelos abaixo assinados:')
signatures(d)
d.add_page_break();d.add_heading('INFORMAÇÕES GERAIS',0)
info(d)
p(d,'Endereço da instituição: Praça Marechal Eduardo Gomes, 50, Vila das Acácias, São José dos Campos/SP, CEP 12228-900.')
p(d,'CNPJ: não informado na proposta; a confirmar pela instituição.')
p(d,'Contato do supervisor: ljr@ita.br | (12) 3947-6946.')
p(d,'Contato da supervisora no ITA: juliana@ita.br | ramal 5980.')
p(d,'Modalidade: estágio curricular obrigatório.')
p(d,'Carga horária total: 225 horas, distribuídas no período de 29/06/2026 a 08/10/2026.')
p(d,'O término previsto originalmente para 28/08/2026 foi estendido para 08/10/2026, mantendo-se a carga total de 225 horas. A folha de frequência apresenta uma distribuição proposta de 45 jornadas de 5 horas, a conferir com o estagiário e validar pelo supervisor.')
d.add_page_break()
sections=[
('I. INTRODUÇÃO',[
'Este relatório apresenta as atividades de desenvolvimento realizadas no LAB-C2DC no âmbito do estágio curricular obrigatório em Engenharia de Computação. O trabalho teve como objetivo construir uma plataforma modular para simulação de serviços de UAS Traffic Management (UTM), com foco no gerenciamento de conflitos de aeronaves não tripuladas em ambientes de baixa altitude.',
'A proposta contemplou rastreamento, previsão de trajetórias, detecção e resolução de conflitos, desconflitamento estratégico e tático e apoio à decisão. O produto desenvolvido é um ambiente computacional extensível, capaz de integrar diferentes algoritmos e registrar execuções para posterior análise. O relato das entregas baseia-se na implementação, no histórico de desenvolvimento, nos notebooks e nos arquivos de resultados disponíveis no projeto.']),
('II. A EMPRESA',[]),
('II.1. Histórico',[
'O estágio foi realizado no Laboratório de Comando e Controle e Defesa Cibernética (LAB-C2DC), no Instituto Tecnológico de Aeronáutica, em São José dos Campos. No contexto descrito pela proposta de estágio, o laboratório é o ambiente institucional de pesquisa e desenvolvimento ao qual se vincula o trabalho de simulação de serviços UTM. A proposta não apresenta datas de criação ou histórico institucional detalhado do laboratório.']),
('II.2. Área onde foi desenvolvido o programa de estágio',[
'O programa concentrou-se em desenvolvimento de software e experimentação computacional para gestão de tráfego de UAS. As atividades envolveram modelagem de aeronaves e espaço aéreo, comunicação entre serviços, gestão de planos de voo e integração de algoritmos de rastreamento e resolução de conflitos. A supervisão na instituição foi atribuída ao professor Lourenço Alves Pereira Júnior, com orientação no ITA da professora Juliana de Melo Bezerra.']),
('II.3. O Estágio no Contexto da Empresa',[
'A plataforma fornece uma base técnica reutilizável para estudos do LAB-C2DC. A separação entre infraestrutura de simulação e algoritmos permite investigar estratégias alternativas sem reescrever os modelos de comunicação, aeronaves ou armazenamento. Os cenários e registros produzidos apoiam a análise de decisões e de efeitos operacionais, contribuindo para futuras pesquisas e protótipos.']),
('III. ATIVIDADES DESENVOLVIDAS',[]),
('III.1. Resumo do Estágio',[
'O trabalho resultou no pacote Python utm_simulator, organizado em componentes de núcleo, domínio, espaço aéreo, aeronaves, serviços UTM, algoritmos e simulação. O histórico do projeto registra a estrutura inicial em julho, evolução dos modelos e módulos em julho e setembro e consolidação da execução, persistência e organização do pacote entre o fim de setembro e outubro.',
'Foram desenvolvidos gestão de planos de voo, troca de mensagens com identificação de correlação, dinâmica e guiagem de UAV, representação tridimensional do espaço aéreo, obstáculos, geofences, regiões de vento e modelos de sensores. Foram integrados módulos de desconflitamento estratégico, rastreamento e desconflitamento tático, com interfaces para substituição dos algoritmos.',
'A etapa de consolidação incluiu registro incremental de resultados em SQLite, visualizações de cenários e trajetórias, configuração de atrasos de comunicação e processamento e documentação prática em notebook. Também foram preparados cenários comparativos com 100 UAVs e diferentes combinações dos serviços estratégico e tático.']),
('III.2. Descrição conceitual de métodos, ferramentas e recursos estudados/usados no estágio',[
'Python e arquitetura modular. O simulador utiliza classes abstratas para definir contratos de dinâmica, guiagem, sensores, rastreamento, previsão e resolução de conflitos e tempo de processamento. Os objetos de domínio incluem waypoints, planos de voo e relatórios de rastreamento. A organização como pacote instalável e o uso de anotações de tipos tornam mais explícitas as responsabilidades e os dados trocados entre componentes.',
'Simulação a eventos discretos. A biblioteca SimPy coordena processos, eventos e avanço do tempo simulado. A rede de comunicação transporta mensagens entre UAVs e serviços UTM e permite representar atraso de transmissão. Os tempos de processamento são modelados nos pontos de execução dos algoritmos, permitindo estudar o efeito de latências no fluxo de decisão.',
'Modelagem de voo e espaço aéreo. A guiagem por waypoints produz comandos para acompanhamento dos planos de voo, enquanto a dinâmica de primeira ordem representa a evolução do movimento. O espaço aéreo inclui geometrias tridimensionais de caixas e cilindros, obstáculos e restrições temporárias. Regiões de vento e sensores perfeitos ou com ruído gaussiano permitem compor condições nominais e não nominais.',
'Gerenciamento estratégico e tático. O serviço estratégico avalia planos antes da partida e pode aprovar, rejeitar ou apresentar uma contraproposta. O rastreamento recebe observações, mantém histórico e produz previsões utilizadas na etapa tática. Esta etapa detecta conflitos durante a execução e permite atualizar planos ativos. Implementações experimentais exercitam alterações de altitude, deslocamentos temporais e desvios laterais, além de validação de candidatos de manobra.',
'Persistência e análise. SQLite armazena estados, trajetórias, mensagens, planos e objetos do espaço aéreo. NumPy apoia operações numéricas; Matplotlib e recursos de imagem permitem gerar gráficos e animações. A separação entre gravação e visualização permite analisar arquivos de resultados após a execução e reduzir a dependência de manter todo o histórico em memória.']),
('III.3. Participação em treinamentos',[
'O projeto inclui um notebook de demonstração do uso do simulador, empregado como material de documentação prática. O código e os exemplos evidenciam estudo aplicado de Python, SimPy, modelagem de serviços UTM e análise de resultados. Não foram identificados registros de cursos, certificados ou treinamentos formais específicos que permitam relacioná-los neste relatório.']),
('III.4. Experimentos, resultados e entregas',[
'Foram preparados exemplos de acompanhamento de trajetórias e cenários de integração dos serviços UTM. O notebook de simulação com 100 UAVs gera rotas entre bordas de uma região retangular, varia altitudes e horários de partida e introduz cinco geofences durante os voos. A geração com semente configurável permite reproduzir a demanda.',
'O cenário compara quatro configurações: sem desconflitamento, apenas estratégico, apenas tático e estratégico combinado com tático. Cada configuração recebe uma simulação independente com a mesma demanda e produz um arquivo SQLite próprio. Os arquivos de resultados dessas configurações estão presentes no projeto.',
'A rotina de análise contempla violações de separação, eventos de colisão conforme os critérios computacionais do cenário, entradas em geofences, intervenções e indicadores de eficiência baseados nas trajetórias. Ocorrências contínuas são agrupadas em eventos, evitando contagem por amostra. O relatório não atribui ganhos percentuais ou garantias de segurança às estratégias, pois essas conclusões exigem validação dos resultados, dos parâmetros e de múltiplos cenários.',
'As entregas técnicas compreendem o pacote modular, algoritmos e interfaces extensíveis, modelos de espaço aéreo e UAV, registros persistentes, exemplos de uso, notebooks de experimentação e visualizações estáticas e animadas. Esses artefatos constituem uma base de pesquisa, sem caracterizar uma implantação operacional de serviço UTM.']),
('IV. COMENTÁRIOS E CONCLUSÕES',[
'O estágio permitiu aplicar conhecimentos de Engenharia de Computação à construção de uma plataforma de simulação para serviços UTM. A principal contribuição foi integrar modelos e serviços em uma arquitetura que permite variar algoritmos, condições ambientais e parâmetros de execução, mantendo registros para análise posterior.',
'O desenvolvimento reforçou competências em programação orientada a objetos, definição de interfaces, simulação a eventos discretos, persistência de dados, análise de trajetórias e organização de experimentos. A evolução do código também evidenciou a importância de contratos claros, controle de estados dos planos de voo e separação entre lógica de decisão e infraestrutura.',
'Como continuidade, recomenda-se ampliar a verificação dos contratos e transições de estado, consolidar testes automatizados, repetir experimentos com múltiplas sementes e condições de tráfego e comparar estratégias com critérios e parâmetros explicitamente documentados. A plataforma permanece como base extensível para pesquisa e desenvolvimento no LAB-C2DC.'])]
for h,paras in sections:
 d.add_heading(h,1 if h.startswith(('I. ','II. ','III. ','IV. ')) else 2)
 for txt in paras:p(d,txt)
d.save(OUT/'Relatorio_Final_Estagio_Caio_Bianchi.docx')

# Frequência: conserva topologia mensal de 31 dias, sem criar presença ou horários não comprovados.
f=doc(9)
months={6:'JUNHO',7:'JULHO',8:'AGOSTO',9:'SETEMBRO',10:'OUTUBRO'}
start=datetime.date(2026,6,29);end=datetime.date(2026,10,8)
for month,label in months.items():
 if month!=6:f.add_page_break()
 title(f,'FOLHA DE FREQUÊNCIA DE ESTÁGIO CURRICULAR')
 p(f,f'MÊS: {label} | ANO: 2026',True)
 p(f,'NOME: '+NAME);p(f,'CURSO: '+COURSE);p(f,'LOCAL DO ESTÁGIO: '+LAB)
 p(f,'Período global: '+PERIOD)
 t=f.add_table(rows=1,cols=6);t.style='Table Grid'
 for c,v in zip(t.rows[0].cells,['DIA','PERÍODO','Nº DE HORAS','DIA','PERÍODO','Nº DE HORAS']):c.text=v
 for i in range(1,17):
  row=t.add_row()
  for idx,day in [(0,i),(3,i+16)]:
   if day>31:continue
   row.cells[idx].text=f'{day:02d}'
   if day>calendar.monthrange(2026,month)[1]:
    row.cells[idx+1].text='Não se aplica';row.cells[idx+2].text='—'
   else:
    date=datetime.date(2026,month,day)
    if date<start or date>end:
     row.cells[idx+1].text='Fora do período';row.cells[idx+2].text='—'
    else:
     if date in SCHEDULE:
      row.cells[idx+1].text='13h às 18h';row.cells[idx+2].text='5'
     else:
      row.cells[idx+1].text='Sem jornada';row.cells[idx+2].text='—'
 for row in t.rows:
  for c in row.cells:
   for x in c.paragraphs:
    x.paragraph_format.space_after=Pt(2);x.paragraph_format.space_before=Pt(2)
    for r in x.runs:r.font.size=Pt(8)
 total=sum(hours for date,hours in SCHEDULE.items() if date.month==month)
 p(f,f'TOTAL DE HORAS DO MÊS: {total} h | TOTAL DO ESTÁGIO: 225 h',True)
 p(f,'Distribuição proposta a pedido do estagiário: três jornadas semanais de 5 h, das 13h às 18h. Datas e horários sujeitos à conferência e validação; esta versão não atesta presença sem assinatura.')
 p(f,'Data da validação: ____/____/________')
 signatures(f)
 p(f,'Supervisor da instituição: assinatura e carimbo.\nSeção de Coordenação e Controle — Divisão de Assuntos Estudantis.')
f.save(OUT/'Folha_de_Frequencia_Caio_Bianchi.docx')
print('Gerados os três arquivos DOCX.')
