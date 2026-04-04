#ifndef TINY_KEPLER_H
#define TINY_KEPLER_H
#include <ctype.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#ifndef MAX_TOKENS
  #define MAX_TOKENS   1024
#endif
#ifndef MAX_ENTITIES
  #define MAX_ENTITIES 1024
#endif
#ifndef MAX_EVENTS
  #define MAX_EVENTS   1024
#endif

#ifndef ALIGN
  #define ALIGN 16
#endif

#define PUSH_ALIGN(offset, align) (offset + (align - 1)) & (~(align-1))

const double __G = 6.6743e-20; // km3/kg/s2

//====================
//
//    Arena
//
//====================

typedef struct {
  char *base;
  size_t offset;
  size_t capacity;
} Arena;

Arena arena_create(size_t capacity);

void arena_destroy(Arena *a);

char *arena_alloc(Arena *a, size_t size);

//===============
//
// Tokenization
//
//===============

typedef enum {
	TOKEN_ID,
	TOKEN_INT,
	TOKEN_DOUBLE,
	TOKEN_LPAREN,
	TOKEN_RPAREN,
	TOKEN_COMMA,
	TOKEN_END,
} token_t;

typedef struct {
	token_t type;
	union {
		struct {
			char *value;
		} id;
		struct {
			int value;
		} intg;
		struct {
			double value;
		} dbl;
	} as;
} Token;

typedef struct {
	const char *content;
	size_t pointer;
	size_t line;
	size_t content_size;
	Token *tokens;
	size_t num_tokens;
} Lexer;

void token_to_str(Token *tok, char* str);
int is_numeric_token(Token *t);
void print_token(Token *tok);
Token eat_id(Arena *a, Lexer *lex);
Token eat_number(Arena *a, Lexer *lex);
void tokenize_mission(Arena *a, Lexer *lex);

//=========================
//
// Mission description
//
//=========================

typedef enum {
	MISSION_CR3BP
} mission_t;

typedef enum {
	ENTITY_SPACECRAFT,
	ENTITY_BODY,
} entity_t;

typedef struct {
	char   *name;
	double args[8];
	size_t num_args;
} FunctionCall;

typedef enum {
	COORD_SPEC_NONE,
	COORD_SPEC_EXPLICIT,
	COORD_SPEC_FUNCTION_CALL,
} coord_spec_t;

typedef struct {
	coord_spec_t type;
	union {
		struct {
			double x, y, z;
		} explicit;
		FunctionCall function_call;
	} as;
} CoordSpec;


typedef struct {
	entity_t  type;
	size_t    id;
	double    x,  y, z;
	double    vx, vy, vz;
	CoordSpec pos_spec, vel_spec;
	union {
		struct {
			char   *name;
			double mass;
			double radius;
			double orbital_radius;
		} body;
		struct {
			double mass;
		} spacecraft;
	} as;
} Entity;

typedef enum {
	ACTION_DRAW,
	ACTION_MANEUVER,
} action_t;

typedef enum {
	DIR_PROGRADE,
	DIR_RETROGRADE,
} direction_t;

typedef enum {
	TRIGGER_NAME,
	TRIGGER_FUNCTION_CALL,
} trigger_t;

typedef enum {
	AT_TIME,
	AT_NAME,
} at_trigger_t;

typedef struct {
	trigger_t type;
	union {
		struct {
			char *value;
		} name;
		struct {
			FunctionCall function_call;
		} function_call;
	} as;
} Trigger;


typedef struct {
	action_t type;
	union {
		struct {
			FunctionCall target;
			int persistent;
		} draw;
		struct {
			double delta_v;
			direction_t direction;
		} maneuver;
	} as;
} Action;

typedef enum {
	EVENT_AT,
	EVENT_WHEN,
} event_t;

typedef enum {
	EVENT_ONCE,
	EVENT_ALWAYS,
} event_periodicity_t;


typedef struct {
	event_t type;
	int     consumed;
	union {
		struct {
			at_trigger_t trigger_type;
			union {
				char *name;
				double t;
			} trigger;
			Action action;
		} at;
		struct {
			Trigger trigger;
			Action action;
			event_periodicity_t periodicity;
		} when;
	} as;
} Event;

typedef enum {
	INTEGRATOR_VERLET,
	INTEGRATOR_RK4,
} integrator_t;

typedef struct {
	mission_t    type;
	double       simulation_time;
	double       dt;
	integrator_t integrator;
	Entity       *entities;
	size_t       num_entities;
	size_t       *entity_id_to_idx;
	Event        *events;
	size_t       num_events;
} MissionDescription;
void mission_type_string(mission_t type, char *buf);
void entity_type_string(entity_t type, char *buf);

// Specs

void integrator_type_string(integrator_t type, char *buf);
void print_entity(Entity *entt);
void print_action(Action *action);
void print_event(Event *event);
void print_mission(MissionDescription *mission);

//=========================
//
// Mission parsing
//
//=========================

typedef struct {
	Token *tokens;
	size_t num_tokens;
	size_t pointer;
	MissionDescription mission;
} MissionParser;

Token *peek(MissionParser *parser);
Token *lookahead(MissionParser *parser);
Token *eat_token(MissionParser *parser, token_t expected);
int eat_int(MissionParser *parser);
double eat_double(MissionParser *parser);
void parse_sim_type(MissionParser *parser);
void parse_sim_time(MissionParser *parser);
void parse_sim_dt(MissionParser *parser);
void parse_integrator(MissionParser *parser);
void parse_entities(MissionParser *parser);
void set_coordinates(
		double x, double y, double z,
		double *px, double *py, double *pz);

FunctionCall parse_function_call(MissionParser *parser);
void parse_initial_state(MissionParser *parser);
void parse_maneuver(MissionParser *parser, Action *action);
void parse_draw(MissionParser *parser, Action *action);

Action parse_action(MissionParser *parser);

Event parse_at_event(MissionParser *parser);
Event parse_when_event(MissionParser *parser);

void parse_events(MissionParser *parser);
void evaluate_function(FunctionCall *fn, MissionDescription *mission, Entity *e);
void evaluate_references(MissionDescription *mission);

MissionDescription parse_mission(Arena *a, Token *tokens, size_t num_tokens);
MissionDescription read_mission(Arena *a, const char *path);

//====================
//
//    ODE
//
//====================

typedef void ode_fun_t(double t, double *y, double *dydt, void *ctx);
typedef void ode_stepper_t(double t, double dt, double *y, double *dydt,
                           ode_fun_t fun, size_t dim_y, void *ctx,
                           double *scratch);

void sum_arrays(double *out, double *a, double *b, size_t num_elems);
void scalar_prod(double *out, double *a, double k, size_t num_elems);

void euler_step(double t, double dt, double *y, ode_fun_t fun, size_t dim_y,
                void *ctx, double *scratch);

void rk4_step(double t, double dt, double *y, ode_fun_t fun, size_t dim_y,
              void *ctx, double *scratch);

// accel_fn: only computes acceleration from position
typedef void accel_fn_t(double *pos, double *accel, void *ctx);
void verlet_step(double t, double dt, double *y, accel_fn_t accel,
                 size_t n_dimensions, void *ctx, double *scratch);

//========================
//
//    Orbital mechanics
//
//========================

#define SQR(x) ((x*x))

double circular_orbital_velocity(double mu, double r);

typedef struct {
	double mu;
} GravParams;

void gravity3d(double t, double *y, double *dydt, GravParams *p);

typedef struct {
	double mu;
} CR3BPParams;

void cr3bp(double t, double *y, double *dydt, void *cr3bp_params);

//========================
//
//    Missions
//
//========================

// ----- Result types -----

typedef struct {
	size_t num_results;
	double mu;
	double L;
	double T;
	double *t;
	double *x;
	double *y;
	double *z;
	double *vx;
	double *vy;
	double *vz;
} CR3BPResults;


typedef struct {
	double mu;
	double L;
	double T;
	size_t primary_id;
} CR3BPNormalization;

// ----- General interface -----

typedef struct {
    mission_t type;
    union {
        CR3BPResults cr3bp;
    } as;
} SimResults;

SimResults run_mission(Arena *a, MissionDescription *mission);

// ----- Classical restricted 3-body problem -----

CR3BPResults run_cr3bp(Arena *a, MissionDescription *mission);
void dump_cr3bp_result(const char *path, CR3BPResults *results);
void read_cr3bp_result(Arena *a, const char *path, CR3BPResults *results);

#ifdef TINY_KEPLER_IMPLEMENTATION
#ifndef TINY_LA_IMPLEMENTATION
	#define TINY_LA_IMPLEMENTATION
#endif

#include <tinyla.h>

//====================
//
//    Arena
//
//====================

Arena arena_create(size_t capacity) {
  Arena a = {0};
  a.capacity = capacity;
  a.base = (char *)malloc(capacity);
  return a;
}

void arena_destroy(Arena *a) { free(a->base); }

char *arena_alloc(Arena *a, size_t size) {
  size_t aligned_offset = PUSH_ALIGN(a->offset, ALIGN);
  if (aligned_offset + size > a->capacity) {
		fprintf(stderr, "Could not allocate memory\n");
    return NULL;
  }
  char *addr = a->base + aligned_offset;
  a->offset = aligned_offset + size;
  return addr;
}

//===============
//
// Tokenization
//
//===============

void token_to_str(Token *tok, char* str){
	switch(tok->type){
		case TOKEN_ID:     { sprintf(str, "TOKEN_ID: %s", tok->as.id.value); break;      }
		case TOKEN_INT:    { sprintf(str, "TOKEN_INT: %d", tok->as.intg.value); break;   }
		case TOKEN_DOUBLE: { sprintf(str, "TOKEN_DOUBLE: %g", tok->as.dbl.value); break; }
		case TOKEN_LPAREN: { sprintf(str, "TOKEN_LPAREN: ("); break;                     }
		case TOKEN_RPAREN: { sprintf(str, "TOKEN_RPAREN: )"); break;                     }
		case TOKEN_COMMA:  { sprintf(str, "TOKEN_COMMA: ,"); break;                      }
		case TOKEN_END:    { sprintf(str, "TOKEN_END"); break;                           }
		default:           { sprintf(str, "Unknown token");                              }
	}
}

int is_numeric_token(Token *t){
	return t->type == TOKEN_INT || t->type == TOKEN_DOUBLE;
}


void print_token(Token *tok){
	char buf[128];
	token_to_str(tok, buf);
	printf("%s\n", buf);
}

Token eat_id(Arena *a, Lexer *lex){
	Token t = { 0 };
	t.type = TOKEN_ID;
	size_t start = lex->pointer;
	while(
			isalnum(lex->content[lex->pointer]) ||
			lex->content[lex->pointer] == '_'
			){
		lex->pointer++;
	}

	size_t id_size = lex->pointer - start;
	t.as.id.value = arena_alloc(a, id_size + 1);
	memcpy(t.as.id.value, lex->content + start, id_size);
	t.as.id.value[id_size] = '\0';
	return t;
}

Token eat_number(Arena *a, Lexer *lex){
	Token t = { 0 };
	t.type = TOKEN_ID;
	size_t start = lex->pointer;
	int has_leading_sign = 0;
	int is_sci= 0;
	int has_sci_leading_sign = 0;
	int is_decimal = 0;
	char current;
	while(1){
		current = lex->content[lex->pointer];
		if(current == '-' || current == '+'){
			if(!has_leading_sign){
				has_leading_sign = 1;
				lex->pointer++;
				continue;
			}else if(lex->content[lex->pointer-1] == 'e'){
				if(!has_sci_leading_sign){
					lex->pointer++;
					continue;
				}
				// Illegal
				break;
			}
			// Illegal
			break;
		}
		if(current == '.'){
			if(!is_decimal){
				is_decimal = 1;
				lex->pointer++;
				continue;
			}
			// Illegal
			break;
		}
		if(current == 'e'){
			if(!is_sci){
				is_sci= 1;
				lex->pointer++;
				continue;
			}
			// Illegal
			break;
		}
		if(!isdigit(current)){
			break;
		}
		lex->pointer++;
	}

	size_t num_size = lex->pointer - start;
	char num_buffer[1024];
	memcpy(num_buffer, lex->content + start, num_size);
	num_buffer[num_size] = '\0';
	if(is_sci || is_decimal){
		t.type = TOKEN_DOUBLE;
		t.as.dbl.value = strtod(num_buffer, NULL);
	}else{
		t.type = TOKEN_INT;
		t.as.intg.value = atoi(num_buffer);
	}

	return t;
}

void tokenize_mission(Arena *a, Lexer *lex){
	while(lex->pointer < lex->content_size){
		char current = lex->content[lex->pointer];
		switch(current){
			case '/':{
				while(
						lex->content[lex->pointer] != '\n' && 
						lex->content[lex->pointer] != '\r'
						){
					lex->pointer++;
				}
				break;
			}
			case '(':{
				Token tok;
				tok.type = TOKEN_LPAREN;
				lex->tokens[lex->num_tokens++] = tok;
				lex->pointer++;
				break;
			}
			case ')':{
				Token tok;
				tok.type = TOKEN_RPAREN;
				lex->tokens[lex->num_tokens++] = tok;
				lex->pointer++;
				break;
			}
			case ',':{
				Token tok;
				tok.type = TOKEN_COMMA;
				lex->tokens[lex->num_tokens++] = tok;
				lex->pointer++;
				break;
			}
			case '\n':{
				lex->line++;
				lex->pointer++;
				break;
			}
			case '\r':{
				lex->line++;
				lex->pointer++;
				break;
			}
			default: {
				if(isalpha(current)){
					Token id = eat_id(a, lex);
					lex->tokens[lex->num_tokens++] = id;
					break;
				}else if(isdigit(current) || 
						(current == '-' && isdigit(lex->content[lex->pointer + 1]))
				){
					Token num = eat_number(a, lex);
					lex->tokens[lex->num_tokens++] = num;
					break;
				}
				lex->pointer++;
		  }
		}

	}
}

//=========================
//
// Mission description
//
//=========================

void mission_type_string(mission_t type, char *buf){
	switch(type){
		case MISSION_CR3BP: { sprintf(buf, "%s", "MISSION_CR3BP"); break; }
		default: { sprintf(buf, "%s", "Unknown"); break; }
	}
}

void entity_type_string(entity_t type, char *buf){
	switch(type){
		case ENTITY_SPACECRAFT: { sprintf(buf, "%s", "ENTITY_SPACECRAFT"); break; }
		case ENTITY_BODY: { sprintf(buf, "%s", "ENTITY_BODY"); break; }
		default: { sprintf(buf, "%s", "Unknown"); break; }
	}
}

// Specs

void integrator_type_string(integrator_t type, char *buf){
	switch(type){
		case INTEGRATOR_VERLET: { sprintf(buf, "%s", "INTEGRATOR_VERLET"); break; }
		case INTEGRATOR_RK4:    { sprintf(buf, "%s", "INTEGRATOR_RK4"); break; }
		default:                { sprintf(buf, "%s", "Unknown"); break; }
	}
}

void print_entity(Entity *entt){
	char buf[128];
	printf("-----  [%zu] -----\n", entt->id);
	entity_type_string(entt->type, buf);
	printf("Type: %s\n", buf);
	if(entt->type == ENTITY_BODY){
		printf("Name: %s\n", entt->as.body.name);
		printf("Mass: %g\n", entt->as.body.mass);
		printf("Radius: %g\n", entt->as.body.radius);
		if(entt->as.body.orbital_radius > 0){
			printf("Orbital radius: %g\n", entt->as.body.orbital_radius);
		}
		printf("Position: (%g, %g, %g)\n", entt->x, entt->y, entt->z);
		printf("Velocity: (%g, %g, %g)\n", entt->vx, entt->vy, entt->vz);
	}
	if(entt->type == ENTITY_SPACECRAFT){
		printf("Mass: %g\n", entt->as.spacecraft.mass);
		printf("Position: (%g, %g, %g)\n", entt->x, entt->y, entt->z);
		printf("Velocity: (%g, %g, %g)\n", entt->vx, entt->vy, entt->vz);
	}
}

void print_action(Action *action){
	switch(action->type){
		case ACTION_DRAW:{
			printf("  Action: DRAW %s(", 
				action->as.draw.target.name);
			for(size_t i = 0; i < action->as.draw.target.num_args; i++){
				printf("%g%s", action->as.draw.target.args[i],
					i < action->as.draw.target.num_args - 1 ? ", " : "");
			}
			printf(")\n");
			break;
		}
		case ACTION_MANEUVER:{
			printf("  Action: MANEUVER DELTA_V %g %s\n",
				action->as.maneuver.delta_v,
				action->as.maneuver.direction == DIR_PROGRADE ? "PROGRADE" : "RETROGRADE");
			break;
		}
	}
}

void print_event(Event *event){
	switch(event->type){
		case EVENT_AT:{
			printf("AT ");
			if(event->as.at.trigger_type == AT_TIME){
				printf("%g", event->as.at.trigger.t);
			}else{
				printf("%s", event->as.at.trigger.name);
			}
			printf("\n");
			print_action(&event->as.at.action);
			break;
		}
		case EVENT_WHEN:{
			printf("WHEN ");
			if(event->as.when.trigger.type == TRIGGER_NAME){
				printf("%s", event->as.when.trigger.as.name.value);
			}else{
				FunctionCall *fn = &event->as.when.trigger.as.function_call.function_call;
				printf("%s(", fn->name);
				for(size_t i = 0; i < fn->num_args; i++){
					printf("%g%s", fn->args[i],
						i < fn->num_args - 1 ? ", " : "");
				}
				printf(")");
			}
			printf(" %s\n", event->as.when.periodicity == EVENT_ONCE ? "ONCE" : "ALWAYS");
			print_action(&event->as.when.action);
			break;
		}
	}
}

void print_mission(MissionDescription *mission){
	char buf[128];
	mission_type_string(mission->type, buf);
	printf("Simulation type: %s\n", buf);
	printf("Simulation time: %f\n", mission->simulation_time);
	printf("Simulation dt: %f\n", mission->dt);
	integrator_type_string(mission->integrator, buf);
	printf("Integrator: %s\n", buf);
	printf("Entities (%zu):\n", mission->num_entities);
	for(size_t i = 0; i < mission->num_entities; i++){
		print_entity(&mission->entities[i]);
	}
	printf("================\n");
	printf("Events (%zu):\n", mission->num_events);
	for(size_t i = 0; i < mission->num_events; i++){
		print_event(&mission->events[i]);
	}
}

//=========================
//
// Mission parsing
//
//=========================

Token *peek(MissionParser *parser){
	return &parser->tokens[parser->pointer];
}

Token *lookahead(MissionParser *parser){
	return &parser->tokens[parser->pointer + 1];
}

Token *eat_token(MissionParser *parser, token_t expected){
	Token *tok = &parser->tokens[parser->pointer];
	if(tok->type != expected){
		char exp_str[128];
		char tok_str[128];
		Token exp = {.type = expected};
		token_to_str(tok, tok_str);
		token_to_str(&exp, exp_str);
		fprintf(stderr, "Expected token of type %s, got %s\n", exp_str, tok_str);
		exit(1);
	}
	parser->pointer++;
	return tok;
}

int eat_int(MissionParser *parser){
	Token *tok = &parser->tokens[parser->pointer];
	if(tok->type != TOKEN_INT){
		char exp_str[128];
		char tok_str[128];
		Token exp = {.type = TOKEN_INT};
		token_to_str(tok, tok_str);
		token_to_str(&exp, exp_str);
		fprintf(stderr, "Expected token of type %s, got %s\n", exp_str, tok_str);
		exit(1);
	}
	parser->pointer++;
	return tok->as.intg.value;
}

double eat_double(MissionParser *parser){
	Token *tok = &parser->tokens[parser->pointer];
	if(!is_numeric_token(tok)){
		char exp_str[128];
		char tok_str[128];
		Token exp = {.type = TOKEN_DOUBLE};
		token_to_str(tok, tok_str);
		token_to_str(&exp, exp_str);
		fprintf(stderr, "Expected a numeric token, got %s\n", tok_str);
		exit(1);
	}
	parser->pointer++;
	if(tok->type == TOKEN_INT){
		return tok->as.intg.value;
	}
	return tok->as.dbl.value;
}

void parse_sim_type(MissionParser *parser){
	eat_token(parser, TOKEN_ID);
	Token *current = &parser->tokens[parser->pointer];
	if(strcmp(current->as.id.value, "CR3BP") == 0){
		parser->mission.type = MISSION_CR3BP;
		eat_token(parser, TOKEN_ID);
	}else{
		fprintf(stderr, "Simulation type not supported: %s\n", current->as.id.value);
		exit(1);
	}
}

void parse_sim_time(MissionParser *parser){
	eat_token(parser, TOKEN_ID);
	Token *current = &parser->tokens[parser->pointer];
	if(current->type == TOKEN_INT){
		parser->mission.simulation_time = current->as.intg.value;
		eat_token(parser, TOKEN_INT);
		return;
	}
	if(current->type == TOKEN_DOUBLE){
		parser->mission.simulation_time = current->as.dbl.value;
		eat_token(parser, TOKEN_DOUBLE);
		return;
	}
	fprintf(stderr, "Invalid simulation time\n");
	exit(1);
}

void parse_sim_dt(MissionParser *parser){
	eat_token(parser, TOKEN_ID);
	double dt = eat_double(parser);
	parser->mission.dt = dt;
}

void parse_integrator(MissionParser *parser){
	eat_token(parser, TOKEN_ID);
	Token *integrator = eat_token(parser, TOKEN_ID);
	const char *integrator_type = integrator->as.id.value;
	if(strcmp(integrator_type, "VERLET") == 0){
		parser->mission.integrator = INTEGRATOR_VERLET;
	}else if (strcmp(integrator_type, "RK4") == 0) {
		parser->mission.integrator = INTEGRATOR_RK4;
	}else{
		fprintf(stderr, "Invalid integrator %s\n", integrator_type);
		exit(1);
	}
}


void parse_entities(MissionParser *parser){
	// Add spacecraft
	Entity e = {0};
	e.type = ENTITY_SPACECRAFT;
	parser->mission.entities[parser->mission.num_entities] = e;
	parser->mission.entity_id_to_idx[e.id] = parser->mission.num_entities;
	parser->mission.num_entities++;

	eat_token(parser, TOKEN_ID);
	Token *current = &parser->tokens[parser->pointer];
	while(
			current->type == TOKEN_ID && 
			(strcmp(current->as.id.value, "BODY") == 0)
	){
		Entity e = {0};
		e.type = ENTITY_BODY;
		eat_token(parser, TOKEN_ID); // BODY
		e.id = eat_int(parser);      // id
		Token *name = eat_token(parser, TOKEN_ID);
		e.as.body.name   = name->as.id.value;
		e.as.body.mass   = eat_double(parser);
		e.as.body.radius = eat_double(parser);
		if(is_numeric_token(peek(parser))){
			e.as.body.orbital_radius = eat_double(parser);
		}else{
			e.as.body.orbital_radius = -1;
		}
		parser->mission.entities[parser->mission.num_entities] = e;
		parser->mission.entity_id_to_idx[e.id] = parser->mission.num_entities;
		parser->mission.num_entities++;
		current = &parser->tokens[parser->pointer];
	}
}

void set_coordinates(
		double x, double y, double z,
		double *px, double *py, double *pz){
	*px = x;
	*py = y;
	*pz = z;
}

FunctionCall parse_function_call(MissionParser *parser){
	Token *function_name = eat_token(parser, TOKEN_ID);
	eat_token(parser, TOKEN_LPAREN);

	FunctionCall fn = {0};
	fn.num_args = 0;
	fn.name = function_name->as.id.value;
	if(peek(parser)->type == TOKEN_RPAREN){
		eat_token(parser, TOKEN_RPAREN);
		// Early return, no args
		return fn;
	}
	while(peek(parser)->type != TOKEN_RPAREN){
		double arg = eat_double(parser);
		fn.args[fn.num_args++] = arg;
		if(peek(parser)->type == TOKEN_COMMA){
			eat_token(parser, TOKEN_COMMA);
		}
	}
	eat_token(parser, TOKEN_RPAREN);
	return fn;
}

void parse_initial_state(MissionParser *parser){
	eat_token(parser, TOKEN_ID);
	Token *current = &parser->tokens[parser->pointer];
	while(
			current->type == TOKEN_ID && 
			(
			 (strcmp(current->as.id.value, "POS") == 0) ||
			 (strcmp(current->as.id.value, "VEL") == 0)
			)
	){
		Token *prefix = eat_token(parser, TOKEN_ID);
		size_t id  = eat_int(parser);
		size_t idx = parser->mission.entity_id_to_idx[id];
		Entity *e  = &parser->mission.entities[idx];

		if(is_numeric_token(peek(parser))){
			double val_x = eat_double(parser);
			double val_y = eat_double(parser);
			double val_z = eat_double(parser);
			if(strcmp(prefix->as.id.value, "POS") == 0){
				e->pos_spec.type = COORD_SPEC_EXPLICIT;
				set_coordinates(
						val_x,
						val_y,
						val_z,
						&e->pos_spec.as.explicit.x,
						&e->pos_spec.as.explicit.y,
						&e->pos_spec.as.explicit.z);
			}else if(strcmp(prefix->as.id.value, "VEL") == 0){
				e->vel_spec.type = COORD_SPEC_EXPLICIT;
				set_coordinates(
						val_x,
						val_y,
						val_z,
						&e->vel_spec.as.explicit.x,
						&e->vel_spec.as.explicit.y,
						&e->vel_spec.as.explicit.z);
			}

		}else{
			FunctionCall function_call = parse_function_call(parser);
			if(strcmp(prefix->as.id.value, "POS") == 0){
				e->pos_spec.type = COORD_SPEC_FUNCTION_CALL;
				e->pos_spec.as.function_call = function_call;
			}
			if(strcmp(prefix->as.id.value, "VEL") == 0){
				e->vel_spec.type = COORD_SPEC_FUNCTION_CALL;
				e->vel_spec.as.function_call = function_call;
			}
		}
		current = &parser->tokens[parser->pointer];
	}
}

void parse_maneuver(MissionParser *parser, Action *action){
	action->type = ACTION_MANEUVER;
	Token *maneuver_type = eat_token(parser, TOKEN_ID);

	if(strcmp(maneuver_type->as.id.value, "DELTA_V") == 0){
		action->as.maneuver.delta_v = eat_double(parser);

		Token *direction = eat_token(parser, TOKEN_ID);
		if(strcmp(direction->as.id.value, "PROGRADE") == 0){
			action->as.maneuver.direction = DIR_PROGRADE;
		}else if(strcmp(direction->as.id.value, "RETROGRADE") == 0){
			action->as.maneuver.direction = DIR_RETROGRADE;
		}else{
			fprintf(stderr, "Unknown maneuver direction: %s\n", direction->as.id.value);
			exit(1);
		}

	}else{
		fprintf(stderr, "Unknown maneuver type: %s\n", maneuver_type->as.id.value);
		exit(1);
	}
}

void parse_draw(MissionParser *parser, Action *action){
	action->type = ACTION_DRAW;
	FunctionCall function_call = parse_function_call(parser);
	action->as.draw.target = function_call;
	action->as.draw.persistent = 1;
}

Action parse_action(MissionParser *parser){
	Action action = {0};
	Token *name = eat_token(parser, TOKEN_ID);
	if(strcmp(name->as.id.value, "MANEUVER") == 0){
		parse_maneuver(parser, &action);
	}else if(strcmp(name->as.id.value, "DRAW") == 0){
		parse_draw(parser, &action);
	}else{
		fprintf(stderr, "Unknown action type: %s\n", name->as.id.value);
		exit(1);
	}
	return action;
}

Event parse_at_event(MissionParser *parser){
	Event ev = {0};
	ev.type = EVENT_AT;

	// Time

	if(peek(parser)->type == TOKEN_ID){
		Token *next = eat_token(parser, TOKEN_ID);
		if(strcmp(next->as.id.value, "START") == 0){
			ev.as.at.trigger_type = AT_NAME;
			ev.as.at.trigger.name = next->as.id.value;
		}else{
			fprintf(stderr, "Unknown event trigger name: %s\n", next->as.id.value);
			exit(1);
		}
	}else{
		double t = eat_double(parser);
		ev.as.at.trigger_type = AT_TIME;
		ev.as.at.trigger.t = t;
	}

	// Action

	Action a = parse_action(parser);
	ev.as.at.action = a;
	return ev;
}

Event parse_when_event(MissionParser *parser){
	Event ev = {0};
	ev.type = EVENT_WHEN;

	// Trigger
	// Check if it's a function call
	if(peek(parser)->type == TOKEN_ID && lookahead(parser)->type == TOKEN_LPAREN){
		FunctionCall function_call = parse_function_call(parser);
		ev.as.when.trigger.type = TRIGGER_FUNCTION_CALL;
		ev.as.when.trigger.as.function_call.function_call = function_call;
	}else{
		Token *name = eat_token(parser, TOKEN_ID);
		ev.as.when.trigger.type = TRIGGER_NAME;
		ev.as.when.trigger.as.name.value = name->as.id.value;
	}

	// Periodicity

	Token *periodicity = eat_token(parser, TOKEN_ID);
	if(strcmp(periodicity->as.id.value, "ONCE") == 0){
		ev.as.when.periodicity = EVENT_ONCE;
	}else if(strcmp(periodicity->as.id.value, "ALWAYS") == 0){
		ev.as.when.periodicity = EVENT_ALWAYS;
	}

	Action action = parse_action(parser);
	ev.as.when.action = action;

	return ev;

}

void parse_events(MissionParser *parser){
	eat_token(parser, TOKEN_ID);
	Token *current = &parser->tokens[parser->pointer];
	while(parser->pointer < parser->num_tokens){
		Token *type = eat_token(parser, TOKEN_ID);
		if(strcmp(type->as.id.value, "AT") == 0){
			Event ev = parse_at_event(parser);
			parser->mission.events[parser->mission.num_events++] = ev;
		}else if(strcmp(type->as.id.value, "WHEN") == 0){
			Event ev = parse_when_event(parser);
			parser->mission.events[parser->mission.num_events++] = ev;
		}else{
			fprintf(stderr, "Unknown event type: %s\n", type->as.id.value);
			exit(1);
		}
	}
}

void evaluate_function(FunctionCall *fn, MissionDescription *mission, Entity *e){
	if(strcmp(fn->name, "CIRCULAR") == 0){

		int craft_id  = fn->args[0];
		int ref_id   = fn->args[1];
		Entity *craft = &mission->entities[mission->entity_id_to_idx[craft_id]];
		Entity *ref  = &mission->entities[mission->entity_id_to_idx[ref_id]];

		double mu = __G * ref->as.body.mass;

		double r_coords[3] = {craft->x, craft->y, craft->z};
		tla_Vector r_craft = {.size = 3, .values=r_coords};
		double r = tla_vector_norm(&r_craft);
		double v = circular_orbital_velocity(mu, r);


		craft->vx = -craft->y/r * v;
		craft->vy = craft->x/r * v;
		craft->vz = 0.0;

		return;
	}
}

void evaluate_references(MissionDescription *mission){
	for(size_t i = 0; i < mission->num_entities; i++){
		Entity *e = &mission->entities[i];

		// Explicit coordinates
		if(e->pos_spec.type == COORD_SPEC_EXPLICIT){
			e->x = e->pos_spec.as.explicit.x;
			e->y = e->pos_spec.as.explicit.y;
			e->z = e->pos_spec.as.explicit.z;
		}
		if(e->vel_spec.type == COORD_SPEC_EXPLICIT){
			e->vx = e->vel_spec.as.explicit.x;
			e->vy = e->vel_spec.as.explicit.y;
			e->vz = e->vel_spec.as.explicit.z;
		}

		// Function-defined coordinates
		if(e->pos_spec.type == COORD_SPEC_FUNCTION_CALL){
			evaluate_function(&e->pos_spec.as.function_call, mission, e);
		}
		if(e->vel_spec.type == COORD_SPEC_FUNCTION_CALL){
			evaluate_function(&e->vel_spec.as.function_call, mission, e);
		}

	}

	for(size_t i = 0; i < mission->num_events; i++){
		Event *e = &mission->events[i];
		if(e->type == EVENT_AT){
			if(e->as.at.trigger_type == AT_NAME){
				if(strcmp(e->as.at.trigger.name, "START") == 0){
					e->as.at.trigger_type = AT_TIME;
					e->as.at.trigger.t = 0.0;
				}
			}
		}

	}
}

MissionDescription parse_mission(Arena *a, Token *tokens, size_t num_tokens){
	MissionParser parser = {0};
	parser.tokens = tokens;
	parser.num_tokens = num_tokens;
	parser.mission.entity_id_to_idx = (size_t*) arena_alloc(a, MAX_ENTITIES * sizeof(size_t));
	parser.mission.entities         = (Entity*) arena_alloc(a, MAX_ENTITIES * sizeof(Entity));
	parser.mission.events           = (Event*) arena_alloc(a, MAX_EVENTS * sizeof(Event));
	while(parser.pointer < num_tokens){
		Token *current = &parser.tokens[parser.pointer];
		if(current->type == TOKEN_ID){
			if(strcmp(current->as.id.value, "SIM_TYPE") == 0){
				parse_sim_type(&parser);
				continue;
			}else if(strcmp(current->as.id.value, "SIM_TIME") == 0){
				parse_sim_time(&parser);
				continue;
			}else if(strcmp(current->as.id.value, "SIM_DT") == 0){
				parse_sim_dt(&parser);
				continue;
			}else if(strcmp(current->as.id.value, "INTEGRATOR") == 0){
				parse_integrator(&parser);
				continue;
			}else if(strcmp(current->as.id.value, "ENTITIES") == 0){
				parse_entities(&parser);
				continue;
			}else if(strcmp(current->as.id.value, "INITIAL_STATE") == 0){
				parse_initial_state(&parser);
				continue;
			}else if(strcmp(current->as.id.value, "EVENTS") == 0){
				parse_events(&parser);
				continue;
			}
		}
		parser.pointer++;	
	}
	evaluate_references(&parser.mission);
	return parser.mission;
}

MissionDescription read_mission(Arena *a, const char *path){
	FILE *f = fopen(path, "rb");
	if(!f){
		fprintf(stderr, "Could not find mission file: '%s'\n", path);
		exit(1);
	}

	fseek(f, 0, SEEK_END);
	size_t size = ftell(f);
	rewind(f);

	char *mission = arena_alloc(a, size + 1); // Null terminator!
	fread(mission, 1, size, f);
	mission[size] = '\0';

	Lexer lex = {0};
	lex.content = mission;
	lex.content_size = size;
	lex.line = 1;
	lex.tokens = (Token*) arena_alloc(a, MAX_TOKENS * sizeof(Token));
	tokenize_mission(a, &lex);

	MissionDescription mission_des = parse_mission(a, lex.tokens, lex.num_tokens);

	fclose(f);
	return mission_des;
}

//====================
//
//    ODE
//
//====================

void sum_arrays(double *out, double *a, double *b, size_t num_elems) {
  for (size_t i = 0; i < num_elems; i++) {
    out[i] = a[i] + b[i];
  }
}
void scalar_prod(double *out, double *a, double k, size_t num_elems) {
  for (size_t i = 0; i < num_elems; i++) {
    out[i] = a[i] * k;
  }
}

void euler_step(double t, double dt, double *y, ode_fun_t fun, size_t dim_y,
                void *ctx, double *scratch) {
  fun(t, y, scratch, ctx);
  for (size_t i = 0; i < dim_y; i++) {
    y[i] += dt * scratch[i];
  }
}

void rk4_step(double t, double dt, double *y, ode_fun_t fun, size_t dim_y,
              void *ctx, double *scratch) {

  double *k1 = scratch;
  double *k2 = scratch + dim_y;
  double *k3 = scratch + 2 * dim_y;
  double *k4 = scratch + 3 * dim_y;
  double *ytmp = scratch + 4 * dim_y;

  fun(t, y, k1, ctx);

  scalar_prod(ytmp, k1, dt / 2, dim_y);
  sum_arrays(ytmp, y, ytmp, dim_y);
  fun(t + dt / 2, ytmp, k2, ctx);

  scalar_prod(ytmp, k2, dt / 2, dim_y);
  sum_arrays(ytmp, y, ytmp, dim_y);
  fun(t + dt / 2, ytmp, k3, ctx);

  scalar_prod(ytmp, k3, dt, dim_y);
  sum_arrays(ytmp, y, ytmp, dim_y);
  fun(t + dt, ytmp, k4, ctx);
  for (size_t i = 0; i < dim_y; i++) {
    y[i] += dt / 6 * (k1[i] + 2 * k2[i] + 2 * k3[i] + k4[i]);
  }
}

// accel_fn: only computes acceleration from position
void verlet_step(double t, double dt, double *y, accel_fn_t accel,
                 size_t n_dimensions, void *ctx, double *scratch) {
  double *pos = y;
  double *vel = y + n_dimensions;
  double *a = scratch;

  // Fist half-kick
  accel(pos, a, ctx);
  for (size_t i = 0; i < n_dimensions; i++) {
    vel[i] += 0.5 * a[i] * dt;
  }

  // Drift
  for (size_t i = 0; i < n_dimensions; i++) {
    pos[i] += vel[i] * dt;
  }

  // Second half-kick
  accel(pos, a, ctx);
  for (size_t i = 0; i < n_dimensions; i++) {
    vel[i] += 0.5 * a[i] * dt;
  }
}

//========================
//
//    Orbital mechanics
//
//========================

double circular_orbital_velocity(double mu, double r){
	return sqrt(mu / r);
}

void gravity3d(double t, double *y, double *dydt, GravParams *p) {
  // a = -mu/r³ * r

  double x = y[0];
  double y_ = y[1];
  double z = y[2];
  double vx = y[3];
  double vy = y[4];
  double vz = y[5];

  double r3 = pow(x * x + y_ * y_ + z * z, 1.5); // r³ = (r²)^3/2

  dydt[0] = vx;
  dydt[1] = vy;
  dydt[2] = vz;
  dydt[3] = -p->mu / r3 * x;
  dydt[4] = -p->mu / r3 * y_;
  dydt[5] = -p->mu / r3 * z;
}

void cr3bp(double t, double *y, double *dydt, void *cr3bp_params) {
		CR3BPParams *p = cr3bp_params;
    double x  = y[0];
    double y_ = y[1];
    double z  = y[2];
    double vx = y[3];
    double vy = y[4];
    double vz = y[5];

    double dx1  = x + p->mu;
    double dx2  = x - 1.0 + p->mu;
		double r1_3 = pow(dx1*dx1 + y_*y_ + z*z, 1.5);
    double r2_3 = pow(dx2*dx2 + y_*y_ + z*z, 1.5);

    dydt[0] = vx;
    dydt[1] = vy;
    dydt[2] = vz;
    dydt[3] =  2.0*vy + x - (1.0-p->mu)*dx1/r1_3 - p->mu*dx2/r2_3;
    dydt[4] = -2.0*vx + y_ - (1.0-p->mu)*y_/r1_3  - p->mu*y_/r2_3;
    dydt[5] =               - (1.0-p->mu)*z/r1_3   - p->mu*z/r2_3;
}

//========================
//
//    Missions
//
//========================

SimResults run_mission(Arena *a, MissionDescription *mission){
	switch(mission->type){
		case MISSION_CR3BP: {
			CR3BPResults results = run_cr3bp(a, mission);
			return (SimResults){
				.type=MISSION_CR3BP,
				.as={
					.cr3bp=results
				}
			};
		}
		default:{
			char buf[128];
			mission_type_string(mission->type, buf);
			fprintf(stderr, "Could not run mission of type: '%s'\n", buf);
			exit(1);
		}
	}
}

Action *poll_events_cr3bp(
		MissionDescription *mission,
		double state[6],
		double old_state[6],
		double t,
		CR3BPNormalization *norm
		)
{
	for(size_t i = 0; i < mission->num_events; i++){
		Event *e = &mission->events[i];
		if(e->consumed) continue;

		switch(e->type){
			case EVENT_AT: {
				double t_real = t * norm->T;
				if(t_real >= e->as.at.trigger.t && !e->consumed){
					e->consumed = 1;
					return &e->as.at.action;
				}
				break;
		  }
			case EVENT_WHEN: {
				if(e->as.when.trigger.type == TRIGGER_NAME){
					// Nothing here yet
					e->consumed = 1;
					return NULL;

				}else if(e->as.when.trigger.type == TRIGGER_FUNCTION_CALL){

					FunctionCall *fn = &e->as.when.trigger.as.function_call.function_call;
					if(strcmp(fn->name, "PERIAPSIS") == 0){
						// TODO: Check if body_idx is whithin bounds
						int body_id  = fn->args[0];
						int body_idx = mission->entity_id_to_idx[body_id];
						Entity *body = &mission->entities[body_idx];

						double bx = 0.0, by = 0.0;
						if(body_id == norm->primary_id){
							bx = -norm->mu;
						}else{
							bx = 1.0 -norm->mu;
						}

						// At periapsis, vr flips from negative to positive

						double dx_old = old_state[0] - bx;
						double dy_old = old_state[1] - by;
						double dx_new = state[0] - bx;
						double dy_new = state[1] - by;

						double vx_old = old_state[3];
						double vy_old = old_state[4];
						double vx_new = state[3];
						double vy_new = state[4];

						double vr_old = dx_old * vx_old + dy_old * vy_old;
						double vr_new = dx_new * vx_new + dy_new * vy_new;

						int should_fire = vr_old < 0 && vr_new >= 0;
						if(should_fire){
							/* printf("Hit the periapsis check\n"); */
							if(e->as.when.periodicity == EVENT_ONCE){
								e->consumed = 1;
							}
							return &e->as.when.action;
						}else{
							return NULL;
						}
					}
				}
		  }
			default: {
				// Unreachable
				return NULL;
		  }
		}
	}

	

	return NULL;
}

void take_action_cr3bp(
		MissionDescription *mission,
		double state[6],
		double old_state[6],
		Action *action,
		CR3BPNormalization *norm)
{
	switch (action->type) {
		case ACTION_DRAW: { break;}
		case ACTION_MANEUVER: {
			double delta_v        = action->as.maneuver.delta_v;
			direction_t direction = action->as.maneuver.direction;
			double multiplier     = direction == DIR_PROGRADE ? 1.0 : -1.0;
			double delta_v_norm = delta_v / ( norm->L / norm->T ); // Normalized delta_v
			double values[3] = {state[3], state[4], state[5]};
			tla_Vector v = {.size=3, .values=values};
			double v_norm = tla_vector_norm(&v);                   // Current velocity in normalized dimensions
			double new_v = v_norm + multiplier * delta_v_norm;
			tla_vector_scalar_mul(&v, &v, new_v/v_norm);
			state[3] = tla_vector_get_value(&v, 0);
			state[4] = tla_vector_get_value(&v, 1);
			state[5] = tla_vector_get_value(&v, 2);
			break;
		}
		default:{
			// Unreachable
		}
	
	}
}

CR3BPResults run_cr3bp(Arena *a, MissionDescription *mission){

	// Preliminary calculations

	Entity *craft = &mission->entities[mission->entity_id_to_idx[0]];
	Entity *earth = &mission->entities[mission->entity_id_to_idx[1]];
	Entity *moon  = &mission->entities[mission->entity_id_to_idx[2]];

	double L  = moon->as.body.orbital_radius;
	double m1 = earth->as.body.mass;
	double m2 = moon->as.body.mass;
	double M  = m1 + m2;
	double T  = sqrt(L * L * L /(__G * M));
	double mu = m2 / M;

	CR3BPParams params = {
		.mu=mu
	};

	CR3BPNormalization norm = {
		.mu         = mu,
		.T          = T,
		.L          = L,
		.primary_id = m1 > m2 ? earth->id : moon->id
	};

	double t = 0;
	double tmax = mission->simulation_time / T;
	double dt = mission->dt / T;
	double n_timesteps = floor(tmax / dt);

	// Initial state

	double state[6] = {
		craft->x / L - mu,
		craft->y / L,
		craft->z / L,
		craft->vx / (L/T),
		craft->vy / (L/T),
		craft->vz / (L/T),
	};
	double old_state[6] = {};
	memcpy(old_state, state, 6 * sizeof(double));

	// Prepare results

	CR3BPResults results = {0};
	results.mu = mu;
	results.T  = T;
	results.L  = L;
	results.t  = (double*) arena_alloc(a, (n_timesteps + 1) * sizeof(double));
	results.x  = (double*) arena_alloc(a, (n_timesteps + 1) * sizeof(double));
	results.y  = (double*) arena_alloc(a, (n_timesteps + 1) * sizeof(double));
	results.z  = (double*) arena_alloc(a, (n_timesteps + 1) * sizeof(double));
	results.vx = (double*) arena_alloc(a, (n_timesteps + 1) * sizeof(double));
	results.vy = (double*) arena_alloc(a, (n_timesteps + 1) * sizeof(double));
	results.vz = (double*) arena_alloc(a, (n_timesteps + 1) * sizeof(double));
	
	results.t[results.num_results]  = t;
	results.x[results.num_results]  = state[0];
	results.y[results.num_results]  = state[1];
	results.z[results.num_results]  = state[2];
	results.vx[results.num_results] = state[3];
	results.vy[results.num_results] = state[4];
	results.vz[results.num_results] = state[5];
	results.num_results++;

	// Simulation

	double scratch[32];
	size_t i = 0;
	Action *action = NULL;

	// Initial events
	action = poll_events_cr3bp(mission, state, old_state, t, &norm);
	if(action != NULL){
			take_action_cr3bp(mission, state, old_state, action, &norm);
	}
	while(t < tmax){
		rk4_step(t, dt, state, cr3bp, 6, &params, scratch);

		action = poll_events_cr3bp(mission, state, old_state, t + dt, &norm);
		if(action != NULL){
				take_action_cr3bp(mission, state, old_state, action, &norm);
		}

		// Push results
		results.t[results.num_results]  = t + dt;
		results.x[results.num_results]  = state[0];
		results.y[results.num_results]  = state[1];
		results.z[results.num_results]  = state[2];
		results.vx[results.num_results] = state[3];
		results.vy[results.num_results] = state[4];
		results.vz[results.num_results] = state[5];
		results.num_results++;

		memcpy(old_state, state, 6 * sizeof(double));
		t += dt;
	}

	return results;
}

void dump_cr3bp_result(const char *path, CR3BPResults *results){
	FILE *f = fopen(path, "wb");
	if(f == NULL){
		fprintf(stderr, "Could not open file: %s\n", path);
	}

	fprintf(f, "CR3BP\n");
	fprintf(f, "mu %lf\n", results->mu);
	fprintf(f, "L %lf\n", results->L);
	fprintf(f, "T %lf\n", results->T);
	fprintf(f, "%zu\n", results->num_results);
	for(size_t i = 0; i < results->num_results; i++){
		fprintf(
				f, 
				"%g %g %g %g %g %g %g\n", 
				results->t[i],
				results->x[i],
				results->y[i],
				results->z[i],
				results->vx[i],
				results->vy[i],
				results->vz[i]
		);
	}

	fclose(f);
}

void read_cr3bp_result(Arena *a, const char *path, CR3BPResults *results){
	FILE *f = fopen(path, "r");
	if(f == NULL){
		fprintf(stderr, "Could not open file: %s\n", path);
		exit(1);
	}

	char type[256];
	fscanf(f, "%s", type);
	if(strcmp(type, "CR3BP") != 0){
		fprintf(stderr, "Expected CR3BP, found %s\n", type);
		exit(1);
	}

	size_t num_results = 0;
	fscanf(f, "%*s %lf", &results->mu);
	fscanf(f, "%*s %lf", &results->L);
	fscanf(f, "%*s %lf", &results->T);
	fscanf(f, "%zu", &num_results);
	results->num_results = num_results;

	results->t  = (double*) arena_alloc(a, num_results * sizeof(double));
	results->x  = (double*) arena_alloc(a, num_results * sizeof(double));
	results->y  = (double*) arena_alloc(a, num_results * sizeof(double));
	results->z  = (double*) arena_alloc(a, num_results * sizeof(double));
	results->vx = (double*) arena_alloc(a, num_results * sizeof(double));
	results->vy = (double*) arena_alloc(a, num_results * sizeof(double));
	results->vz = (double*) arena_alloc(a, num_results * sizeof(double));

	for(size_t i = 0; i < num_results; i++){
		fscanf(
				f, 
				"%lf %lf %lf %lf %lf %lf %lf", 
				&results->t[i],
				&results->x[i],
				&results->y[i],
				&results->z[i],
				&results->vx[i],
				&results->vy[i],
				&results->vz[i]
		);
	}

	fclose(f);
}

#endif

#endif // TINY_KEPLER_H
