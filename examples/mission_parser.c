#define TINY_KEPLER_IMPLEMENTATION
#define TINY_LA_IMPLEMENTATION
#include <ctype.h>
#include <stddef.h>
#include <string.h>
#include <tinyla.h>
#include <tiny-kepler.h>
#include <stdio.h>
#include <stdlib.h>

#define MAX_TOKENS   1024
#define MAX_ENTITIES 1024
#define MAX_EVENTS   1024

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

typedef struct {
	const char *content;
	size_t pointer;
	size_t line;
	size_t content_size;
	Token *tokens;
	size_t num_tokens;
} Lexer;

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
				printf("Found a comment in position %zu, line %zu\n", lex->pointer, lex->line);
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

	printf("Found %zu tokens.\n", lex->num_tokens);
	for(size_t i = 0; i < lex->num_tokens; i++){
		printf("[%zu] ", i);
		print_token(&lex->tokens[i]);
	}

}

//=========================
//
// Mission description
//
//=========================

typedef enum {
	MISSION_CR3BP
} mission_t;

void mission_type_string(mission_t type, char *buf){
	switch(type){
		case MISSION_CR3BP: { sprintf(buf, "%s", "MISSION_CR3BP"); break; }
		default: { sprintf(buf, "%s", "Unknown"); break; }
	}
}


typedef enum {
	ENTITY_SPACECRAFT,
	ENTITY_BODY,
} entity_t;

void entity_type_string(entity_t type, char *buf){
	switch(type){
		case ENTITY_SPACECRAFT: { sprintf(buf, "%s", "ENTITY_SPACECRAFT"); break; }
		case ENTITY_BODY: { sprintf(buf, "%s", "ENTITY_BODY"); break; }
		default: { sprintf(buf, "%s", "Unknown"); break; }
	}
}

// Specs

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
	union {
		struct {
			at_trigger_t trigger_type;
			union {
				double t;
				char *name;
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

typedef struct {
	mission_t type;
	double    simulation_time;
	Entity    *entities;
	size_t    num_entities;
	size_t    *entity_id_to_idx;
	Event     *events;
	size_t    num_events;
} MissionDescription;

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
	printf("----- /[%zu] -----\n", entt->id);
}

void print_mission(MissionDescription *mission){
	char buf[128];
	mission_type_string(mission->type, buf);
	printf("Simulation type: %s\n", buf);
	printf("Simulation time: %f\n", mission->simulation_time);
	printf("Entities (%zu):\n", mission->num_entities);
	for(size_t i = 0; i < mission->num_entities; i++){
		print_entity(&mission->entities[i]);
	}
}

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

Action parse_action(MissionParser *parser){
	Action action = {0};
	Token *name = eat_token(parser, TOKEN_ID);
	if(strcmp(name->as.id.value, "MANEUVER") == 0){
		parse_maneuver(parser, &action);
	}else{
		fprintf(stderr, "Unknown action type: %s\n", name->as.id.value);
		exit(1);
	}
	return action;
}

Event parse_at_event(MissionParser *parser){
	Event ev = {0};
	ev.type = EVENT_AT;

	Token *next = peek(parser);
	if(next->type == TOKEN_ID){
		if(strcmp(next->as.id.value, "START") == 0){
		}else{
			fprintf(stderr, "Unknown event type: %s\n", next->as.id.value);
			exit(1);
		}
	}else{
		double time = eat_double(parser);
	}

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

void read_mission(Arena *a, const char *path){
	FILE *f = fopen(path, "rb");
	if(!f){
		fprintf(stderr, "Could not find mission file: '%s'\n", path);
		exit(1);
	}

	fseek(f, 0, SEEK_END);
	size_t size = ftell(f);
	printf("Mission size: %zu B\n", size);
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
	print_mission(&mission_des);

	fclose(f);
}


int main(){

	Arena a = arena_create(1024 * 1024); // 1MB
	const char *path = "./examples/missions/reduced-3-body.mission";
	read_mission(&a, path);

	arena_destroy(&a);
	return 0;
}
